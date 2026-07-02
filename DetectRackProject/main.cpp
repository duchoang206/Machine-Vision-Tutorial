#include "CameraStream.hpp"
#include "RegionMonitor.hpp"
#include "YOLOv8Detector.hpp"
#include <atomic>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>
#include <set>

// Networking Headers
#include <ixwebsocket/IXWebSocketServer.h>
#include <ixwebsocket/IXWebSocket.h>
#include <ixwebsocket/IXNetSystem.h>
#include <modbus/modbus.h>

#ifdef _WIN32
#include <winsock2.h>
#endif

// --- Tọa độ ROI Toàn cục ---
std::vector<cv::Point> g_pts1 = {
    cv::Point(1154, 414),
    cv::Point(1297, 347),
    cv::Point(1440, 406),
    cv::Point(1297, 473)
};
std::vector<cv::Point> g_pts2 = {
    cv::Point(1476, 427),
    cv::Point(1655, 512),
    cv::Point(1520, 575),
    cv::Point(1341, 490)
};

// --- Trạng thái vi phạm ROI (Toàn cục) ---
std::mutex g_alarmMutex;
bool roi_1_alarm = false;
bool roi_2_alarm = false;

// --- Vùng đệm chia sẻ (Shared Frame Buffer) (Camera -> AI & WS) ---
std::mutex g_bufferMutex;
cv::Mat g_sharedFrame;
bool g_hasNewFrame = false;

// --- Vùng đệm hiển thị GUI cục bộ ---
std::mutex g_guiMutex;
cv::Mat g_guiFrame;
bool g_guiNewFrame = false;

// --- Quản lý các kết nối WebSocket client ---
std::mutex g_wsClientsMutex;
std::set<std::shared_ptr<ix::WebSocket>> g_wsClients;

std::atomic<bool> g_running(true);

// Kiểm tra xem vị trí phát hiện có nằm trong phân khu đặt rack hợp lệ hay không
bool isValidRackArea(const cv::Rect &box) {
  // Tránh khu vực di chuyển của AGV bên trái (x < 700) và vạch kẻ sọc phía trước (y + h > 600)
  return box.x >= 700 && (box.y + box.height) <= 600;
}

// Tăng cường độ tương phản cục bộ để làm nổi bật vân kim loại của rack
cv::Mat enhanceContrast(const cv::Mat &src) {
  if (src.empty())
    return src;
  cv::Mat lab, dst;
  cv::cvtColor(src, lab, cv::COLOR_BGR2Lab);
  std::vector<cv::Mat> planes(3);
  cv::split(lab, planes);
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(4.0, cv::Size(8, 8));
  clahe->apply(planes[0], planes[0]);
  cv::merge(planes, lab);
  cv::cvtColor(lab, dst, cv::COLOR_Lab2BGR);
  return dst;
}

// Hàm gửi báo cáo lên Server khi trạng thái thay đổi
void reportToServer(const std::string &roiName, bool hasRack) {
  std::cout << "\n[SERVER REPORT] " << roiName << " state changed: "
            << (hasRack ? "RACK DETECTED (OCCUPIED)" : "NO RACK (EMPTY)")
            << std::endl;
}

// Cấu trúc dùng làm công cụ phụ trợ lấy tọa độ mới bằng chuột
struct MouseCallbackParams {
  cv::Rect box;
  bool drawing = false;
};

void onMouse(int event, int x, int y, int flags, void *userdata) {
  auto *params = reinterpret_cast<MouseCallbackParams *>(userdata);
  if (!params)
    return;
  if (event == cv::EVENT_LBUTTONDOWN) {
    params->drawing = true;
    params->box = cv::Rect(x, y, 0, 0);
  } else if (event == cv::EVENT_MOUSEMOVE && params->drawing) {
    params->box.width = x - params->box.x;
    params->box.height = y - params->box.y;
  } else if (event == cv::EVENT_LBUTTONUP && params->drawing) {
    params->drawing = false;
    if (params->box.width < 0) {
      params->box.x += params->box.width;
      params->box.width = -params->box.width;
    }
    if (params->box.height < 0) {
      params->box.y += params->box.height;
      params->box.height = -params->box.height;
    }
    std::cout << "[Config Helper] Drawn Box: cv::Rect(" << params->box.x << ", "
              << params->box.y << ", " << params->box.width << ", "
              << params->box.height << ")" << std::endl;
  }
}

// Hàm kiểm tra xem có vật thể (Rack) nào nằm trong hình bình hành không
bool checkPolygonIntersection(const std::vector<cv::Point> &polygon,
                              const std::vector<Detection> &detections) {
  if (polygon.empty() || detections.empty())
    return false;

  std::vector<cv::Point2f> polyF;
  polyF.reserve(polygon.size());
  for (const auto &p : polygon) {
    polyF.push_back(cv::Point2f(p.x, p.y));
  }
  double polyArea = cv::contourArea(polyF);
  if (polyArea <= 0)
    return false;

  for (const auto &det : detections) {
    // 1. Kiểm tra nguyên bản: bottomCenter nằm trong hoặc trên cạnh của đa giác
    cv::Point bottomCenter(det.box.x + det.box.width / 2,
                           det.box.y + det.box.height);
    double test = cv::pointPolygonTest(
        polygon, cv::Point2f(bottomCenter.x, bottomCenter.y), false);
    if (test >= 0) {
      return true;
    }

    // 2. Kiểm tra nâng cao bằng diện tích giao cắt
    std::vector<cv::Point2f> rectF = {
        cv::Point2f(det.box.x, det.box.y),
        cv::Point2f(det.box.x + det.box.width, det.box.y),
        cv::Point2f(det.box.x + det.box.width, det.box.y + det.box.height),
        cv::Point2f(det.box.x, det.box.y + det.box.height)
    };

    std::vector<cv::Point2f> intersection;
    float intersectArea = cv::intersectConvexConvex(polyF, rectF, intersection, true);
    
    if ((intersectArea / polyArea) > 0.40f) {
      return true;
    }
  }
  return false;
}

// ==========================================
// THREAD 1: CAMERA GRABBING THREAD
// ==========================================
void cameraThreadFunc(CameraStream* camera) {
  cv::Mat tempFrame;
  while (g_running) {
    if (camera->retrieveFrame(tempFrame)) {
      if (!tempFrame.empty()) {
        std::lock_guard<std::mutex> lock(g_bufferMutex);
        g_sharedFrame = tempFrame.clone(); // Frame Dropping
        g_hasNewFrame = true;
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
}

// ==========================================
// THREAD 2: AI CORE THREAD (Chỉ xử lý AI và Monitor)
// ==========================================
void aiCoreThreadFunc(YOLOv8Detector* detector, RegionMonitor* monitor1, RegionMonitor* monitor2) {
  cv::Mat localFrame;
  bool prevOccupied1 = false;
  bool prevOccupied2 = false;

  while (g_running) {
    bool process = false;
    {
      std::lock_guard<std::mutex> lock(g_bufferMutex);
      if (g_hasNewFrame) {
        localFrame = g_sharedFrame.clone();
        g_hasNewFrame = false;
        process = true;
      }
    }

    if (process && !localFrame.empty()) {
      cv::Mat enhanced = enhanceContrast(localFrame);
      std::vector<Detection> dets = detector->detect(enhanced);

      // Lọc các phát hiện nằm trong vùng đặt rack hợp lệ
      std::vector<Detection> validDets;
      for (const auto &d : dets) {
        if (isValidRackArea(d.box)) {
          validDets.push_back(d);
        }
      }

      // Kiểm tra va chạm đa giác (Hình bình hành)
      bool isOccupied1 = checkPolygonIntersection(g_pts1, validDets);
      bool isOccupied2 = checkPolygonIntersection(g_pts2, validDets);

      monitor1->checkIntersection(validDets);
      monitor2->checkIntersection(validDets);

      // Báo cáo thay đổi trạng thái
      if (isOccupied1 != prevOccupied1) {
        reportToServer("ROI1", isOccupied1);
        prevOccupied1 = isOccupied1;
      }
      if (isOccupied2 != prevOccupied2) {
        reportToServer("ROI2", isOccupied2);
        prevOccupied2 = isOccupied2;
      }

      // Cập nhật 2 biến trạng thái toàn cục chung
      {
        std::lock_guard<std::mutex> lock(g_alarmMutex);
        roi_1_alarm = isOccupied1;
        roi_2_alarm = isOccupied2;
      }

      // Đẩy khung hình thô sang luồng GUI
      {
        std::lock_guard<std::mutex> lock(g_guiMutex);
        g_guiFrame = localFrame.clone();
        g_guiNewFrame = true;
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
}

// ==========================================
// THREAD 3: WEBSOCKET SERVER THREAD (Truyền 30 FPS mượt mà)
// ==========================================
void websocketThreadFunc() {
  ix::WebSocketServer server(8082, "0.0.0.0");

  server.setOnConnectionCallback([](std::weak_ptr<ix::WebSocket> webSocket,
                                    std::shared_ptr<ix::ConnectionState> connectionState) {
    auto ws = webSocket.lock();
    if (ws) {
      {
        std::lock_guard<std::mutex> lock(g_wsClientsMutex);
        g_wsClients.insert(ws);
      }
      std::cout << "[WebSocket] Client connected: " << connectionState->getRemoteIp() << std::endl;

      ws->setOnMessageCallback([webSocket](const ix::WebSocketMessagePtr& msg) {
        if (msg->type == ix::WebSocketMessageType::Close || msg->type == ix::WebSocketMessageType::Error) {
          std::lock_guard<std::mutex> lock(g_wsClientsMutex);
          auto wsShared = webSocket.lock();
          if (wsShared) {
            g_wsClients.erase(wsShared);
          }
          std::cout << "[WebSocket] Client disconnected." << std::endl;
        }
      });
    }
  });

  auto res = server.listen();
  if (!res.first) {
    std::cerr << "[WebSocket] Error starting server: " << res.second << std::endl;
    return;
  }

  server.start();

  double streamFps = 0.0;
  int frameCount = 0;
  auto lastFpsUpdate = std::chrono::steady_clock::now();

  while (g_running) {
    // 1. Lấy trạng thái Alarm hiện tại để đóng gói JSON gửi Web (Dạng TEXT)
    bool r1 = false;
    bool r2 = false;
    {
      std::lock_guard<std::mutex> lock(g_alarmMutex);
      r1 = roi_1_alarm;
      r2 = roi_2_alarm;
    }

    std::string jsonStr = "{\"roi_1_alarm\": " + std::string(r1 ? "true" : "false") +
                          ", \"roi_2_alarm\": " + std::string(r2 ? "true" : "false") + "}";

    // 2. Lấy khung hình mới nhất từ Camera và vẽ ROI để stream nhị phân (Dạng BINARY)
    cv::Mat localFrame;
    {
      std::lock_guard<std::mutex> lock(g_bufferMutex);
      if (!g_sharedFrame.empty()) {
        localFrame = g_sharedFrame.clone();
      }
    }

    std::vector<uchar> localJpeg;
    bool hasFrame = false;

    if (!localFrame.empty()) {
      // Tính toán FPS của luồng truyền phát
      frameCount++;
      auto now = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed = now - lastFpsUpdate;
      if (elapsed.count() >= 0.5) {
        streamFps = frameCount / elapsed.count();
        frameCount = 0;
        lastFpsUpdate = now;
      }

      // Tự vẽ đa giác ROI trực tiếp trong luồng truyền phát ở tần suất 30 FPS mượt mà
      cv::Scalar color1 = r1 ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
      std::vector<std::vector<cv::Point>> polys1 = {g_pts1};
      cv::polylines(localFrame, polys1, true, color1, 2);
      cv::putText(localFrame, "ROI 1", cv::Point(g_pts1[0].x, g_pts1[0].y - 8),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, color1, 2);

      cv::Scalar color2 = r2 ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
      std::vector<std::vector<cv::Point>> polys2 = {g_pts2};
      cv::polylines(localFrame, polys2, true, color2, 2);
      cv::putText(localFrame, "ROI 2", cv::Point(g_pts2[0].x, g_pts2[0].y - 8),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, color2, 2);

      // Hiển thị trạng thái ROI ở góc trên bên trái màn hình stream
      std::string statusText1 = "ROI 1: " + std::string(r1 ? "OCCUPIED" : "SAFE");
      std::string statusText2 = "ROI 2: " + std::string(r2 ? "OCCUPIED" : "SAFE");
      cv::putText(localFrame, statusText1, cv::Point(30, 40), cv::FONT_HERSHEY_SIMPLEX, 0.75, color1, 2);
      cv::putText(localFrame, statusText2, cv::Point(30, 70), cv::FONT_HERSHEY_SIMPLEX, 0.75, color2, 2);

      // Hiển thị FPS lên video stream gửi lên Web
      if (streamFps > 0.0) {
        std::string fpsText = cv::format("FPS: %.1f", streamFps);
        cv::putText(localFrame, fpsText, cv::Point(16, localFrame.rows - 16),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        cv::putText(localFrame, fpsText, cv::Point(15, localFrame.rows - 17),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
      }

      // Downscale ảnh về độ phân giải 960x540 để tối ưu tốc độ mạng và giảm tải CPU nén ảnh JPEG
      cv::Mat webFrame;
      cv::resize(localFrame, webFrame, cv::Size(960, 540));

      std::vector<uchar> buffer;
      std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70}; // Tối ưu chất lượng nén 70%
      cv::imencode(".jpg", webFrame, buffer, params);
      localJpeg = std::move(buffer);
      hasFrame = true;
    }

    // Broadcast tới các client Web
    {
      std::lock_guard<std::mutex> lock(g_wsClientsMutex);
      if (!g_wsClients.empty()) {
        for (auto& ws : g_wsClients) {
          // Gửi text JSON chứa trạng thái alarm
          ws->sendText(jsonStr);
          
          // Gửi ảnh nhị phân JPEG thời gian thực
          if (hasFrame && !localJpeg.empty()) {
            std::string binaryData(reinterpret_cast<char*>(localJpeg.data()), localJpeg.size());
            ws->sendBinary(binaryData);
          }
        }
      }
    }

    // Định kỳ gửi luồng 33ms (~30 FPS)
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
  }

  server.stop();
}

// ==========================================
// THREAD 4: MODBUS TCP SERVER THREAD (Port 502)
// ==========================================
void modbusThreadFunc() {
  modbus_t *ctx = modbus_new_tcp("0.0.0.0", 502);
  if (ctx == NULL) {
    std::cerr << "[Modbus] Unable to create Modbus TCP context." << std::endl;
    return;
  }

  modbus_mapping_t *mb_mapping = modbus_mapping_new(2, 0, 0, 0);
  if (mb_mapping == NULL) {
    std::cerr << "[Modbus] Failed to allocate mapping: " << modbus_strerror(errno) << std::endl;
    modbus_free(ctx);
    return;
  }

  int server_socket = modbus_tcp_listen(ctx, 1);
  if (server_socket == -1) {
    std::cerr << "[Modbus] Listen failed on port 502: " << modbus_strerror(errno) << std::endl;
    modbus_mapping_free(mb_mapping);
    modbus_free(ctx);
    return;
  }

  std::cout << "[Modbus] TCP Server listening on port 502..." << std::endl;

  while (g_running) {
    int client_socket = modbus_tcp_accept(ctx, &server_socket);
    if (client_socket == -1) {
      if (!g_running) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    std::cout << "[Modbus] Client connected." << std::endl;

    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
    while (g_running) {
      modbus_set_response_timeout(ctx, 0, 200000); // 200ms

      int rc = modbus_receive(ctx, query);
      if (rc > 0) {
        {
          std::lock_guard<std::mutex> lock(g_alarmMutex);
          mb_mapping->tab_bits[0] = roi_1_alarm ? 1 : 0;
          mb_mapping->tab_bits[1] = roi_2_alarm ? 1 : 0;
        }
        modbus_reply(ctx, query, rc, mb_mapping);
      } else if (rc == -1) {
        break;
      }
    }
    std::cout << "[Modbus] Client disconnected." << std::endl;
    modbus_close(ctx);
  }

  if (server_socket != -1) {
#ifdef _WIN32
    closesocket(server_socket);
#else
    close(server_socket);
#endif
  }
  modbus_mapping_free(mb_mapping);
  modbus_free(ctx);
}

// ==========================================
// MAIN FUNCTION (GUI / Thread 0 Coordinator)
// ==========================================
int main(int argc, char *argv[]) {
  ix::initNetSystem();

  std::string videoSource = "rtsp://admin:rtc%402025@192.168.5.201:554/cam/realmonitor?channel=1&subtype=0";
  std::string modelPath = "weights/best.onnx";
  if (argc > 1)
    videoSource = argv[1];
  if (argc > 2)
    modelPath = argv[2];
  if (!std::filesystem::exists(modelPath) &&
      std::filesystem::exists("../" + modelPath)) {
    modelPath = "../" + modelPath;
  }

  CameraStream camera(videoSource);
  if (!camera.start()) {
    ix::uninitNetSystem();
    return -1;
  }

  YOLOv8Detector detector(modelPath, cv::Size(640, 640), 0.20f, 0.45f);
  if (!detector.loadModel()) {
    camera.stop();
    ix::uninitNetSystem();
    return -1;
  }

  RegionMonitor monitor1;
  cv::Rect roiRect1 = cv::boundingRect(g_pts1);
  monitor1.handleMouseCallback(cv::EVENT_LBUTTONDOWN, roiRect1.x, roiRect1.y, 0);
  monitor1.handleMouseCallback(cv::EVENT_MOUSEMOVE, roiRect1.x + roiRect1.width, roiRect1.y + roiRect1.height, 0);
  monitor1.handleMouseCallback(cv::EVENT_LBUTTONUP, roiRect1.x + roiRect1.width, roiRect1.y + roiRect1.height, 0);

  RegionMonitor monitor2;
  cv::Rect roiRect2 = cv::boundingRect(g_pts2);
  monitor2.handleMouseCallback(cv::EVENT_LBUTTONDOWN, roiRect2.x, roiRect2.y, 0);
  monitor2.handleMouseCallback(cv::EVENT_MOUSEMOVE, roiRect2.x + roiRect2.width, roiRect2.y + roiRect2.height, 0);
  monitor2.handleMouseCallback(cv::EVENT_LBUTTONUP, roiRect2.x + roiRect2.width, roiRect2.y + roiRect2.height, 0);

  // Khởi động luồng xử lý song song
  std::thread grabThread(cameraThreadFunc, &camera);
  std::thread aiThread(aiCoreThreadFunc, &detector, &monitor1, &monitor2);
  std::thread wsThread(websocketThreadFunc);
  std::thread modbusThread(modbusThreadFunc);

  // Cấu hình giao diện GUI OpenCV cục bộ
  std::string winName = "DetectRackProject - Predefined ROI Demo";
  cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);

  MouseCallbackParams mouseParams;
  cv::setMouseCallback(winName, onMouse, &mouseParams);

  cv::Mat localFrame;

  // Vòng lặp giao diện GUI chính (Thread 0)
  while (g_running) {
    bool hasNewGui = false;
    {
      std::lock_guard<std::mutex> lock(g_guiMutex);
      if (g_guiNewFrame) {
        localFrame = g_guiFrame.clone();
        g_guiNewFrame = false;
        hasNewGui = true;
      }
    }

    if (hasNewGui && !localFrame.empty()) {
      // Lấy trạng thái báo động mới nhất
      bool r1 = false;
      bool r2 = false;
      {
        std::lock_guard<std::mutex> lock(g_alarmMutex);
        r1 = roi_1_alarm;
        r2 = roi_2_alarm;
      }

      // Tự vẽ đa giác ROI cho màn hình OpenCV cục bộ
      cv::Scalar color1 = r1 ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
      std::vector<std::vector<cv::Point>> polys1 = {g_pts1};
      cv::polylines(localFrame, polys1, true, color1, 2);
      cv::putText(localFrame, "ROI 1", cv::Point(g_pts1[0].x, g_pts1[0].y - 8),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, color1, 2);

      cv::Scalar color2 = r2 ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
      std::vector<std::vector<cv::Point>> polys2 = {g_pts2};
      cv::polylines(localFrame, polys2, true, color2, 2);
      cv::putText(localFrame, "ROI 2", cv::Point(g_pts2[0].x, g_pts2[0].y - 8),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, color2, 2);

      // Hiển thị trạng thái ROI ở góc trên bên trái màn hình OpenCV
      std::string statusText1 = "ROI 1: " + std::string(r1 ? "OCCUPIED" : "SAFE");
      std::string statusText2 = "ROI 2: " + std::string(r2 ? "OCCUPIED" : "SAFE");
      cv::putText(localFrame, statusText1, cv::Point(30, 40), cv::FONT_HERSHEY_SIMPLEX, 0.75, color1, 2);
      cv::putText(localFrame, statusText2, cv::Point(30, 70), cv::FONT_HERSHEY_SIMPLEX, 0.75, color2, 2);

      // Vẽ hộp đo đạc phụ trợ chuột (nếu có vẽ)
      if (mouseParams.drawing || (mouseParams.box.width > 0 && mouseParams.box.height > 0)) {
        cv::rectangle(localFrame, mouseParams.box, cv::Scalar(0, 255, 255), 1, cv::LINE_8);
        cv::putText(localFrame, "Temp Box", cv::Point(mouseParams.box.x, mouseParams.box.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
      }
      cv::imshow(winName, localFrame);
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 'q' || key == 'Q' || key == 27) {
      g_running = false;
      break;
    }
  }

  g_running = false;

  std::cout << "[Shutdown] Terminating parallel threads..." << std::endl;
  if (grabThread.joinable()) grabThread.join();
  if (aiThread.joinable()) aiThread.join();
  if (wsThread.joinable()) wsThread.join();
  if (modbusThread.joinable()) modbusThread.join();

  camera.stop();
  cv::destroyAllWindows();
  ix::uninitNetSystem();

  std::cout << "[Shutdown] Cleanup complete." << std::endl;
  return 0;
}

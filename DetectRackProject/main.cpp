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

// ============================================================================
// KHU VỰC CẤU HÌNH HỆ THỐNG - DÀNH CHO KHÁCH HÀNG SỬA IP / CỔNG MẠNG
// ============================================================================

// 1. Địa chỉ luồng RTSP của Camera AI (Khách hàng sửa địa chỉ IP 192.168.5.201 tại đây nếu đổi camera)
const std::string DEFAULT_CAMERA_RTSP = "rtsp://admin:rtc%402025@192.168.5.201:554/cam/realmonitor?channel=1&subtype=0";

// 2. Đường dẫn đến file weights mô hình YOLOv8 (.onnx)
const std::string DEFAULT_MODEL_PATH = "weights/best.onnx";

// 3. Cấu hình cổng kết nối mạng LAN cho thiết bị ngoại vi
const int MODBUS_PORT = 502;          // Cổng Modbus TCP gửi dữ liệu cho PLC/AGV (Mặc định: 502)
const int WEBSOCKET_PORT = 8082;      // Cổng truyền phát video và alarm cho Web Dashboard (Mặc định: 8082)

// 4. Tọa độ các đa giác ROI (Hình bình hành giám sát khu vực đặt Rack)
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

// ============================================================================

// --- Trạng thái vi phạm ROI toàn cục ---
std::mutex g_alarmMutex;
bool roi_1_alarm = false;
bool roi_2_alarm = false;

// --- Vùng đệm chia sẻ (Camera -> AI & WS) ---
std::mutex g_bufferMutex;
cv::Mat g_sharedFrame;
bool g_hasNewFrame = false;

// --- Vùng đệm hiển thị GUI OpenCV cục bộ ---
std::mutex g_guiMutex;
cv::Mat g_guiFrame;
bool g_guiNewFrame = false;

// --- Quản lý các kết nối Web Clients ---
std::mutex g_wsClientsMutex;
std::set<std::shared_ptr<ix::WebSocket>> g_wsClients;

std::atomic<bool> g_running(true);

// Lọc các vùng phát hiện rack hợp lệ (tránh AGV di chuyển ở phần ngoài)
bool isValidRackArea(const cv::Rect &box) {
  return box.x >= 700 && (box.y + box.height) <= 600;
}

// Cải thiện độ tương phản ảnh camera (CLAHE)
cv::Mat enhanceContrast(const cv::Mat &src) {
  if (src.empty()) return src;
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

// Gửi log sự kiện khi trạng thái thay đổi
void reportToServer(const std::string &roiName, bool hasRack) {
  std::cout << "[REPORT] " << roiName << " thay đổi trạng thái: "
            << (hasRack ? "CÓ RACK (OCCUPIED)" : "TRỐNG (EMPTY)")
            << std::endl;
}

// Kiểm tra va chạm giữa đối tượng Rack phát hiện bởi YOLO và vùng đa giác ROI
bool checkPolygonIntersection(const std::vector<cv::Point> &polygon,
                              const std::vector<Detection> &detections) {
  if (polygon.empty() || detections.empty()) return false;

  std::vector<cv::Point2f> polyF;
  polyF.reserve(polygon.size());
  for (const auto &p : polygon) polyF.push_back(cv::Point2f(p.x, p.y));

  double polyArea = cv::contourArea(polyF);
  if (polyArea <= 0) return false;

  for (const auto &det : detections) {
    // Cách 1: Tâm dưới của bounding box nằm trong đa giác
    cv::Point bottomCenter(det.box.x + det.box.width / 2, det.box.y + det.box.height);
    if (cv::pointPolygonTest(polygon, cv::Point2f(bottomCenter.x, bottomCenter.y), false) >= 0) {
      return true;
    }

    // Cách 2: Diện tích giao cắt chiếm tỷ lệ lớn (>40%) vùng ROI
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
// LUỒNG 1: CAMERA GRABBING (Lấy khung hình thô từ Camera)
// ==========================================
void cameraThreadFunc(CameraStream* camera) {
  cv::Mat tempFrame;
  while (g_running) {
    if (camera->retrieveFrame(tempFrame)) {
      if (!tempFrame.empty()) {
        std::lock_guard<std::mutex> lock(g_bufferMutex);
        g_sharedFrame = tempFrame.clone();
        g_hasNewFrame = true;
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
}

// ==========================================
// LUỒNG 2: AI CORE (Chạy mô hình YOLOv8 và kiểm tra ROI)
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

      std::vector<Detection> validDets;
      for (const auto &d : dets) {
        if (isValidRackArea(d.box)) {
          validDets.push_back(d);
        }
      }

      bool isOccupied1 = checkPolygonIntersection(g_pts1, validDets);
      bool isOccupied2 = checkPolygonIntersection(g_pts2, validDets);

      monitor1->checkIntersection(validDets);
      monitor2->checkIntersection(validDets);

      if (isOccupied1 != prevOccupied1) {
        reportToServer("ROI1", isOccupied1);
        prevOccupied1 = isOccupied1;
      }
      if (isOccupied2 != prevOccupied2) {
        reportToServer("ROI2", isOccupied2);
        prevOccupied2 = isOccupied2;
      }

      {
        std::lock_guard<std::mutex> lock(g_alarmMutex);
        roi_1_alarm = isOccupied1;
        roi_2_alarm = isOccupied2;
      }

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
// LUỒNG 3: WEBSOCKET STREAMING (Phát 30 FPS lên Web Dashboard)
// ==========================================
void websocketThreadFunc() {
  ix::WebSocketServer server(WEBSOCKET_PORT, "0.0.0.0");

  server.setOnConnectionCallback([](std::weak_ptr<ix::WebSocket> webSocket,
                                    std::shared_ptr<ix::ConnectionState> connectionState) {
    auto ws = webSocket.lock();
    if (ws) {
      {
        std::lock_guard<std::mutex> lock(g_wsClientsMutex);
        g_wsClients.insert(ws);
      }
      std::cout << "[WebSocket] Thiết bị kết nối: " << connectionState->getRemoteIp() << std::endl;

      ws->setOnMessageCallback([webSocket](const ix::WebSocketMessagePtr& msg) {
        if (msg->type == ix::WebSocketMessageType::Close || msg->type == ix::WebSocketMessageType::Error) {
          std::lock_guard<std::mutex> lock(g_wsClientsMutex);
          auto wsShared = webSocket.lock();
          if (wsShared) g_wsClients.erase(wsShared);
          std::cout << "[WebSocket] Thiết bị ngắt kết nối." << std::endl;
        }
      });
    }
  });

  auto res = server.listen();
  if (!res.first) {
    std::cerr << "[WebSocket] Lỗi khởi động server: " << res.second << std::endl;
    return;
  }

  server.start();

  double streamFps = 0.0;
  int frameCount = 0;
  auto lastFpsUpdate = std::chrono::steady_clock::now();

  while (g_running) {
    bool r1 = false, r2 = false;
    {
      std::lock_guard<std::mutex> lock(g_alarmMutex);
      r1 = roi_1_alarm;
      r2 = roi_2_alarm;
    }

    std::string jsonStr = "{\"roi_1_alarm\": " + std::string(r1 ? "true" : "false") +
                          ", \"roi_2_alarm\": " + std::string(r2 ? "true" : "false") + "}";

    cv::Mat localFrame;
    {
      std::lock_guard<std::mutex> lock(g_bufferMutex);
      if (!g_sharedFrame.empty()) localFrame = g_sharedFrame.clone();
    }

    std::vector<uchar> localJpeg;
    bool hasFrame = false;

    if (!localFrame.empty()) {
      // Tính toán FPS của luồng
      frameCount++;
      auto now = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed = now - lastFpsUpdate;
      if (elapsed.count() >= 0.5) {
        streamFps = frameCount / elapsed.count();
        frameCount = 0;
        lastFpsUpdate = now;
      }

      // Vẽ đa giác ROI lên ảnh truyền phát
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

      // Trạng thái góc trái
      std::string statusText1 = "ROI 1: " + std::string(r1 ? "OCCUPIED" : "SAFE");
      std::string statusText2 = "ROI 2: " + std::string(r2 ? "OCCUPIED" : "SAFE");
      cv::putText(localFrame, statusText1, cv::Point(30, 40), cv::FONT_HERSHEY_SIMPLEX, 0.75, color1, 2);
      cv::putText(localFrame, statusText2, cv::Point(30, 70), cv::FONT_HERSHEY_SIMPLEX, 0.75, color2, 2);

      // Hiển thị FPS
      if (streamFps > 0.0) {
        std::string fpsText = cv::format("FPS: %.1f", streamFps);
        cv::putText(localFrame, fpsText, cv::Point(16, localFrame.rows - 16),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        cv::putText(localFrame, fpsText, cv::Point(15, localFrame.rows - 17),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
      }

      // Nén ảnh chất lượng cao 960x540
      cv::Mat webFrame;
      cv::resize(localFrame, webFrame, cv::Size(960, 540));
      std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
      cv::imencode(".jpg", webFrame, localJpeg, params);
      hasFrame = true;
    }

    // Broadcast tới các client
    {
      std::lock_guard<std::mutex> lock(g_wsClientsMutex);
      if (!g_wsClients.empty()) {
        for (auto& ws : g_wsClients) {
          ws->sendText(jsonStr);
          if (hasFrame && !localJpeg.empty()) {
            std::string binaryData(reinterpret_cast<char*>(localJpeg.data()), localJpeg.size());
            ws->sendBinary(binaryData);
          }
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
  }

  server.stop();
}

// ==========================================
// LUỒNG 4: MODBUS TCP SERVER (Gửi cảnh báo sang PLC)
// ==========================================
void modbusThreadFunc() {
  modbus_t *ctx = modbus_new_tcp("0.0.0.0", MODBUS_PORT);
  if (ctx == NULL) {
    std::cerr << "[Modbus] Không thể tạo Modbus TCP context." << std::endl;
    return;
  }

  modbus_mapping_t *mb_mapping = modbus_mapping_new(2, 0, 0, 0);
  if (mb_mapping == NULL) {
    std::cerr << "[Modbus] Lỗi cấp phát bộ nhớ Modbus." << std::endl;
    modbus_free(ctx);
    return;
  }

  int server_socket = modbus_tcp_listen(ctx, 1);
  if (server_socket == -1) {
    std::cerr << "[Modbus] Lỗi lắng nghe cổng " << MODBUS_PORT << std::endl;
    modbus_mapping_free(mb_mapping);
    modbus_free(ctx);
    return;
  }

  std::cout << "[Modbus] Server đang lắng nghe ở cổng " << MODBUS_PORT << "..." << std::endl;

  while (g_running) {
    int client_socket = modbus_tcp_accept(ctx, &server_socket);
    if (client_socket == -1) {
      if (!g_running) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    std::cout << "[Modbus] Đã kết nối thiết bị điều khiển." << std::endl;

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
    std::cout << "[Modbus] Thiết bị điều khiển ngắt kết nối." << std::endl;
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

// ============================================================================
// HÀM CHÍNH (Đóng vai trò khởi tạo hệ thống và quản lý giao diện OpenCV cục bộ)
// ============================================================================
int main(int argc, char *argv[]) {
  ix::initNetSystem();

  std::string videoSource = DEFAULT_CAMERA_RTSP;
  std::string modelPath = DEFAULT_MODEL_PATH;
  if (argc > 1) videoSource = argv[1];
  if (argc > 2) modelPath = argv[2];

  if (!std::filesystem::exists(modelPath) && std::filesystem::exists("../" + modelPath)) {
    modelPath = "../" + modelPath;
  }

  // Khởi động camera
  CameraStream camera(videoSource);
  if (!camera.start()) {
    ix::uninitNetSystem();
    return -1;
  }

  // Khởi động bộ dò tìm YOLOv8
  YOLOv8Detector detector(modelPath, cv::Size(640, 640), 0.20f, 0.45f);
  if (!detector.loadModel()) {
    camera.stop();
    ix::uninitNetSystem();
    return -1;
  }

  // Khởi tạo vùng giám sát cho RegionMonitor từ tọa độ ROI đã khai báo
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

  // Kích hoạt các luồng chạy song song
  std::thread grabThread(cameraThreadFunc, &camera);
  std::thread aiThread(aiCoreThreadFunc, &detector, &monitor1, &monitor2);
  std::thread wsThread(websocketThreadFunc);
  std::thread modbusThread(modbusThreadFunc);

  // Mở cửa sổ OpenCV hiển thị trực tiếp cục bộ tại Server để debug nhanh
  std::string winName = "DetectRackProject - Live Server Debug Monitor";
  cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);

  cv::Mat localFrame;

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
      bool r1 = false, r2 = false;
      {
        std::lock_guard<std::mutex> lock(g_alarmMutex);
        r1 = roi_1_alarm;
        r2 = roi_2_alarm;
      }

      // Vẽ đa giác và hiển thị trạng thái lên cửa sổ GUI cục bộ
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

      std::string statusText1 = "ROI 1: " + std::string(r1 ? "OCCUPIED" : "SAFE");
      std::string statusText2 = "ROI 2: " + std::string(r2 ? "OCCUPIED" : "SAFE");
      cv::putText(localFrame, statusText1, cv::Point(30, 40), cv::FONT_HERSHEY_SIMPLEX, 0.75, color1, 2);
      cv::putText(localFrame, statusText2, cv::Point(30, 70), cv::FONT_HERSHEY_SIMPLEX, 0.75, color2, 2);

      cv::imshow(winName, localFrame);
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 'q' || key == 'Q' || key == 27) {
      g_running = false;
      break;
    }
  }

  g_running = false;

  std::cout << "[Shutdown] Đang dừng tất cả các luồng..." << std::endl;
  if (grabThread.joinable()) grabThread.join();
  if (aiThread.joinable()) aiThread.join();
  if (wsThread.joinable()) wsThread.join();
  if (modbusThread.joinable()) modbusThread.join();

  camera.stop();
  cv::destroyAllWindows();
  ix::uninitNetSystem();

  std::cout << "[Shutdown] Hoàn thành dọn dẹp hệ thống." << std::endl;
  return 0;
}

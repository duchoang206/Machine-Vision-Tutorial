R-SkyView DetectRack
Hệ thống phát hiện kệ hàng (rack) công nghiệp theo thời gian thực, chạy trên kiến trúc Edge AI: xử lý AI và hình học ngay tại máy chủ biên đặt trong xưởng, sau đó phân phối dữ liệu tới Web Dashboard và PLC.
Luồng dữ liệu tổng quát: Camera IP (RTSP) → Edge Server (C++17, YOLOv8) → WebSocket (Web Dashboard) + Modbus TCP (PLC/SCADA).
---
Mục lục
Yêu cầu hệ thống
Cấu trúc mã nguồn
Cấu hình hệ thống
Vận hành & triển khai
Kiến trúc hệ thống (UML)
Ghi chú cho developer
---
1. Yêu cầu hệ thống
Môi trường:
Windows 10/11 hoặc Linux (Ubuntu 20.04+)
Trình biên dịch hỗ trợ chuẩn C++17
CMake ≥ 3.10
Thư viện phụ thuộc:
Thư viện	Vai trò
OpenCV 4.x	Đọc luồng camera, tăng cường ảnh (CLAHE), mã hóa JPEG
libmodbus	Giao tiếp Modbus TCP với PLC/SCADA
ixwebsocket	Phát luồng WebSocket đa luồng, độ trễ thấp (< 30ms)
---
2. Cấu trúc mã nguồn
```text
DetectRackProject/
├── cmake/
│   └── Findlibmodbus.cmake     # Module CMake để tìm thư viện Modbus
├── include/
│   ├── CameraStream.hpp        # Quản lý luồng camera RTSP
│   ├── RegionMonitor.hpp       # Giám sát logic vùng ROI
│   └── YOLOv8Detector.hpp      # Khởi tạo model, dự đoán bounding box
├── src/
│   ├── CameraStream.cpp
│   ├── RegionMonitor.cpp
│   └── YOLOv8Detector.cpp
├── weights/
│   └── best.onnx                # Trọng số YOLOv8 đã export ONNX
├── CMakeLists.txt
├── main.cpp                     # Điều phối chính, nạp cấu hình, quản lý luồng
└── index.html                   # Web Dashboard giám sát thời gian thực
```
---
3. Cấu hình hệ thống
Toàn bộ tham số mạng, camera và vùng giám sát được gom vào struct `SystemConfig` ở đầu `main.cpp`. Khi triển khai ở xưởng mới, chỉ cần sửa các giá trị trong struct này, không cần đụng vào logic xử lý bên dưới.
```cpp
struct SystemConfig {
    // Mạng & giao thức
    std::string ws_host = "0.0.0.0";     // IP lắng nghe WebSocket
    int ws_port = 8082;                  // Cổng WebSocket cho Web Dashboard

    std::string modbus_ip = "0.0.0.0";   // IP lắng nghe Modbus TCP
    int modbus_port = 502;               // Cổng Modbus TCP chuẩn cho PLC

    // Nguồn dữ liệu & model AI
    std::string video_source = "rtsp://admin:rtc%402025@192.168.5.201:554/...";
    std::string model_path = "weights/best.onnx";

    // Stream Web (tối ưu băng thông)
    int stream_width = 960;
    int stream_height = 540;
    int jpeg_quality = 70;               // Giảm nếu mạng yếu

    // Tọa độ 4 đỉnh vùng ROI (hình bình hành)
    std::vector<cv::Point> roi_pts_1 = {
        cv::Point(1154, 414), cv::Point(1297, 347),
        cv::Point(1440, 406), cv::Point(1297, 473)
    };
    std::vector<cv::Point> roi_pts_2 = {
        cv::Point(1476, 427), cv::Point(1655, 512),
        cv::Point(1520, 575), cv::Point(1341, 490)
    };
};
```
---
4. Vận hành & triển khai
4.1 Chạy Edge AI Server
```bash
# Cú pháp
./DetectRackProject "<URL_RTSP_CAMERA>" "<ĐƯỜNG_DẪN_ONNX>"

# Ví dụ
./DetectRackProject "rtsp://192.168.1.100:554/live" "../weights/best.onnx"
```
Nếu không truyền tham số, chương trình dùng giá trị mặc định trong `SystemConfig`.
4.2 Triển khai Web Dashboard
`index.html` tự lấy IP từ thanh địa chỉ trình duyệt để kết nối ngược vào WebSocket của Edge Server, nên không cần sửa mã nguồn frontend khi đổi mạng.
Kịch bản A — Cùng mạng LAN xưởng:
Lấy IP nội bộ của máy chạy Edge Server (ví dụ `ipconfig` → `192.168.5.101`).
Tại thư mục chứa `index.html`, mở server tĩnh:
```bash
   python -m http.server 8000
   ```
Các máy khác trong mạng truy cập: `http://192.168.5.101:8000`
Kịch bản B — Giám sát từ xa qua Cloud:
Đẩy `index.html` lên GitHub repository.
Deploy tĩnh qua Vercel hoặc Netlify → nhận link công khai (ví dụ `https://detect-rack-dashboard.vercel.app`).
Port forwarding cổng 8082 (WebSocket) trên router xưởng, trỏ về IP nội bộ của Edge Server, để trang web ngoài Internet kết nối được vào luồng dữ liệu.
---
5. Kiến trúc hệ thống (UML)
5.1 Sơ đồ thành phần (Component Diagram)
Luồng dữ liệu từ camera vật lý, qua các module xử lý trong Edge Server, đến các client tiêu thụ dữ liệu.
```mermaid
flowchart LR
    Cam(["RTSP IP Camera"])

    subgraph Edge["Edge AI Server - C++17 Backend"]
        CamMod["CameraStream"]
        AIMod["YOLOv8Detector"]
        ROIMod["RegionMonitor"]
        WSMod["WebSocket Server - port 8082"]
        MBMod["Modbus TCP Server - port 502"]

        CamMod -->|shared frame buffer| AIMod
        AIMod -->|danh sach Detection| ROIMod
        ROIMod -->|JSON + JPEG 30 FPS| WSMod
        ROIMod -->|ghi Alarm Coil| MBMod
    end

    subgraph Clients["External Clients"]
        WebClient["Web Dashboard"]
        PLC["PLC / SCADA"]
    end

    Cam -->|H.264 / H.265 stream| CamMod
    WSMod -.->|WebSocket| WebClient
    MBMod -.->|Modbus TCP| PLC

    style Edge fill:#eef4ff,stroke:#5b7fdb,stroke-width:1px
    style Clients fill:#f4f4f4,stroke:#999999,stroke-width:1px
    style Cam fill:#fff3d6,stroke:#d6a326,stroke-width:1px
```
Ảnh thô từ camera đi qua `CameraStream`, sau đó `YOLOv8Detector` nhận diện vật thể và chuyển tọa độ cho `RegionMonitor` tính giao cắt hình học. Kết quả tách thành hai kênh độc lập: WebSocket cho giao diện Web và Modbus TCP cho PLC.
5.2 Sơ đồ lớp (Class Diagram)
```mermaid
classDiagram
    class SystemConfig {
        +string ws_host
        +int ws_port
        +string modbus_ip
        +int modbus_port
        +string video_source
        +string model_path
        +int stream_width
        +int stream_height
        +int jpeg_quality
        +Point[] roi_pts_1
        +Point[] roi_pts_2
    }

    class CameraStream {
        -string videoSource
        -VideoCapture cap
        +CameraStream(string source)
        +bool start()
        +bool retrieveFrame(Mat frame)
        +void stop()
    }

    class YOLOv8Detector {
        -string modelPath
        -Size inputSize
        -float confThreshold
        -float nmsThreshold
        -Net net
        +YOLOv8Detector(string path, Size size, float conf, float nms)
        +bool loadModel()
        +Detection[] detect(Mat frame)
    }

    class Detection {
        +Rect box
        +float confidence
        +int classId
    }

    class RegionMonitor {
        -Point[] polygon
        -bool isOccupied
        +void setROI(Point[] pts)
        +bool checkIntersection(Detection[] detections)
        +void handleMouseCallback(int event, int x, int y, int flags)
    }

    class Main {
        <<entry point>>
    }

    Main ..> SystemConfig : doc cau hinh
    Main --> CameraStream : quan ly vong doi
    Main --> YOLOv8Detector : dieu phoi suy dien
    Main --> RegionMonitor : khoi tao ROI 1 va 2
    YOLOv8Detector ..> Detection : tao ra
    RegionMonitor ..> Detection : danh gia va cham
```
`SystemConfig` giữ toàn bộ tham số tĩnh; `main()` điều phối vòng đời của `CameraStream`, `YOLOv8Detector` và `RegionMonitor`; `Detection` là đối tượng dữ liệu trung gian truyền giữa các module.
5.3 Sơ đồ trình tự khởi động & đa luồng (Sequence Diagram)
Hệ thống chạy 4 luồng song song độc lập để tránh nghẽn cổ chai giữa xử lý AI và truyền mạng.
```mermaid
sequenceDiagram
    autonumber
    actor Eng as Ky su van hanh
    participant Main as main - Thread 0
    participant Cfg as SystemConfig
    participant Cam as CameraStream - Thread 1
    participant AI as YOLOv8Detector - Thread 2
    participant WS as WebSocket Server - Thread 3
    participant MB as Modbus TCP - Thread 4

    Eng->>Main: Chay ung dung
    Main->>Cfg: Nap cau hinh
    Cfg-->>Main: OK

    Main->>Cam: start()
    Cam-->>Main: Ket noi RTSP thanh cong

    Main->>AI: loadModel()
    AI-->>Main: Nap ONNX hoan tat

    rect rgb(238, 244, 255)
    Note over Main,MB: Khoi chay song song 4 luong
    Main->>Cam: Thread 1 - doc va drop frame vao buffer
    Main->>AI: Thread 2 - CLAHE + YOLOv8 + tinh ROI
    Main->>WS: Thread 3 - nen JPEG + phat JSON qua port 8082
    Main->>MB: Thread 4 - lang nghe port 502, phuc vu PLC
    end

    Main->>Main: Vong lap GUI OpenCV cuc bo - Thread 0
```
Bước 1–7 là khởi tạo và kiểm tra an toàn (camera, model). Sau đó `main()` đẩy 4 tác vụ vào 4 luồng CPU độc lập chạy `while(g_running)`, giúp tốc độ AI không phụ thuộc vào tốc độ truyền mạng.
5.4 Sơ đồ trạng thái vùng giám sát (State Diagram)
```mermaid
stateDiagram-v2
    [*] --> INITIALIZING
    INITIALIZING --> SAFE : Camera OK va Intersection duoi 40%

    SAFE --> ALARM_OCCUPIED : Rack dat vao vung, Intersection tren 40%
    note right of ALARM_OCCUPIED
        Kich hoat 1 lan duy nhat khi chuyen trang thai
        Log SERVER REPORT ra terminal
        Gui JSON bao dong qua WebSocket
        Ghi Modbus Coil = 1
    end note

    ALARM_OCCUPIED --> SAFE : Rack duoc di doi khoi vung
    note right of SAFE
        Reset trang thai
        Log huy su kien ra terminal
        Gui JSON cap nhat SAFE qua WebSocket
        Ghi Modbus Coil = 0
    end note

    SAFE --> [*] : Dung chuong trinh - q hoac Esc
    ALARM_OCCUPIED --> [*] : Dung chuong trinh - q hoac Esc
```
Ngưỡng giao cắt được tính bằng `intersectConvexConvex` giữa đa giác ROI và bounding box. Khi tỷ lệ diện tích giao vượt 40%, hệ thống chuyển từ `SAFE` sang `ALARM_OCCUPIED`. Cơ chế event-driven đảm bảo cảnh báo chỉ phát sinh đúng 1 lần tại thời điểm chuyển trạng thái, tránh spam băng thông và CPU.
---
6. Ghi chú cho developer
Lấy tọa độ ROI mới bằng chuột:
Chương trình có sẵn mouse listener trên cửa sổ OpenCV. Kéo chuột trái để vẽ vùng thử, tọa độ `cv::Rect(x, y, w, h)` sẽ in ra terminal — dùng giá trị này để cập nhật `roi_pts_1` / `roi_pts_2`.
Firewall:
Nếu Dashboard báo `OFFLINE` dù server đang chạy, kiểm tra và mở Inbound Rules cho các cổng:
`8000` — HTTP server giao diện web
`8082` — WebSocket stream hình ảnh
`502` — Modbus TCP tới PLC
Bộ nhớ trình duyệt:
`index.html` gọi `URL.revokeObjectURL()` sau mỗi lần render frame mới, tránh memory leak khi Dashboard chạy liên tục 24/7.
---
R-SkyView Industrial AI Vision — Edge Server v1.0.0
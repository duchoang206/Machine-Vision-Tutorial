# 🚀 Hướng Dẫn Cài Đặt, Vận Hành & Kiến Trúc UML Hệ Thống R-SkyView DetectRack

Tài liệu này cung cấp hướng dẫn chi tiết từ các bước thiết lập môi trường, biên dịch mã nguồn C++, quy trình cấu hình mạng, cho đến việc phân tích kiến trúc chuyên sâu của hệ thống **DetectRackProject** thông qua ngôn ngữ mô hình hóa thống nhất (UML).

Hệ thống hoạt động theo mô hình **Industrial AI Edge-Client**: Máy chủ biên (Edge Server) xử lý các tác vụ nặng (AI & Tính toán hình học) tại biên và phân phối dữ liệu thời gian thực tới giao diện giám sát tĩnh (Web Dashboard) và các thiết bị tự động hóa (PLC).

---

## 📑 MỤC LỤC
1. [Yêu Cầu Hệ Thống (Prerequisites)](#1-yêu-cầu-hệ-thống-prerequisites)
2. [Cấu Trúc Mã Nguồn Bàn Giao](#2-cấu-trúc-mã-nguồn-bàn-giao)
3. [Hướng Dẫn Biên Dịch (Build Project)](#3-hướng-dẫn-biên-dịch-build-project)
4. [Tập Trung Hóa Cấu Hình Hệ Thống (SystemConfig)](#4-tập-trung-hóa-cấu-hình-hệ-thống-systemconfig)
5. [Hướng Dẫn Vận Hành & Triển Khai Mạng Mới](#5-hướng-dẫn-vận-hành--triển-khai-mạng-mới)
6. [Phân Tích Sơ Đồ Luồng UML Chi Tiết](#6-phân-tích-sơ-đồ-luồng-uml-chi-tiết)
7. [Ghi Chú Kỹ Thuật Cho Nhà Phát Triển (Developer Notes)](#7-ghi-chú-kỹ-thuật-cho-nhà-phát-triển-developer-notes)

---

## 1. Yêu Cầu Hệ Thống (Prerequisites)

### Môi trường phát triển & Biên dịch:
* **Hệ điều hành:** Windows 10/11 hoặc Linux (Ubuntu 20.04+).
* **Trình biên dịch:** Hỗ trợ tiêu chuẩn **C++17** (`CMAKE_CXX_STANDARD 17`).
* **Công cụ build:** CMake (Phiên bản $\geq$ 3.10).

### Các thư viện phụ thuộc bắt buộc (Dependencies):
1.  **OpenCV (Phiên bản 4.x):** Xử lý luồng hình ảnh camera, tăng cường độ tương phản cục bộ (CLAHE) và mã hóa nén ảnh JPEG.
2.  **libmodbus:** Xử lý giao thức truyền thông Modbus TCP công nghiệp kết nối tới các thiết bị tự động hóa (PLC/SCADA).
3.  **ixwebsocket:** Thành phần xử lý truyền tải gói tin WebSocket đa luồng tốc độ cao với độ trễ tối thiểu (< 30ms).

---

## 2. Cấu Trúc Mã Nguồn Bàn Giao

```text
DetectRackProject/
├── cmake/
│   └── Findlibmodbus.cmake     # Module cấu hình để CMake tìm kiếm thư viện Modbus
├── include/
│   ├── CameraStream.hpp        # Quản lý luồng Camera RTSP độc lập
│   ├── RegionMonitor.hpp       # Quản lý và giám sát logic vùng ROI
│   └── YOLOv8Detector.hpp      # Khởi tạo mô hình và dự đoán bounding boxes
├── src/
│   ├── CameraStream.cpp
│   ├── RegionMonitor.cpp
│   └── YOLOv8Detector.cpp
├── weights/
│   └── best.onnx               # File trọng số mô hình YOLOv8 đã được tối ưu hóa
├── CMakeLists.txt              # Cấu hình biên dịch tổng thể hệ thống
├── main.cpp                    # Điều phối chính (Nạp cấu hình, quản lý đa luồng)
└── index.html                  # Giao diện Web Dashboard giám sát thời gian thực
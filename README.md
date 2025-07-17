# Camera-Calibrator
roscore
roslaunch usb_cam usb_cam.launch
chay export DISPLAY=:0 (neu dung VNC)
rosrun camera_calibration cameracalibrator.py --size 6x8 --square 0.059 --camera_name usb_cam image:=/usb_cam/image_raw camera:=/usb_cam

✅ 1. Cách cầm bảng hiệu chỉnh
 • Luôn cầm bảng vuông góc với camera, không cần nghiêng.
 • Không quan trọng bảng ngang hay dọc, quan trọng là:
 • Phải thay đổi vị trí, xoay góc, dịch chuyển xa gần, nghiêng trái/phải, để camera thấy các phối cảnh khác nhau của bảng.
 • Ít nhất 15–20 ảnh với các tư thế khác nhau của bảng.
 
 ✅ 2. Khi nào bấm “CALIBRATE”
 • Khi bạn thấy thanh trạng thái ở dưới (thanh màu vàng) báo hiệu là đã thu được đủ số ảnh (ví dụ: Captured: 22 good).
 • Lúc đó, nút CALIBRATE sẽ sáng lên và bạn bấm vào.
 • Nếu chưa đủ ảnh tốt, nút CALIBRATE sẽ bị mờ.
 
 ✅ 3. Sau khi bấm “CALIBRATE”
 • Đợi khoảng vài giây tới vài chục giây để nó tính toán.
 • Sau khi hoàn thành, bấm SAVE để lưu file hiệu chỉnh.
 • Bấm COMMIT để ghi dữ liệu này về ROS (xuất ra file .yaml).
 
 #Chay tu file .bag
 rosrun camera_calibration cameracalibrator.py --size 6x8 --square 0.059 image:=/usb_cam/image_raw

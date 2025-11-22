# SOLAR HÀ GIANG

[http://solarhagiang.vn](http://solarhagiang.vn/)

Gói này được thiết kế để đọc BMS Seplos V3 trong cấu hình multipack. Ở thế hệ V3, BMS đầu tiên hoạt động như một Modbus master trong khi tất cả các BMS khác hoạt động như các slave. Trong cấu hình này, không thể truy cập BMS từ thiết bị thứ ba qua Modbus nữa, vì không thể có hai thiết bị master trong hệ thống RS-485 Modbus. Gói này thụ động phát hiện giao tiếp giữa các thiết bị, điều này không làm gián đoạn giao tiếp của từng BMS.

Trong YAML, tất cả dữ liệu cần thiết phải được chèn/sửa đổi. Cứ sau 200 ms, BMS sẽ truyền một tập dữ liệu mới. Khoảng thời gian cập nhật có thể được thay đổi (mặc định: 5 giây).

![seplos 4x](https://github.com/user-attachments/assets/9d710287-069d-44b6-acda-e96764642a33)

Để thiết lập kết nối, chân 1/8 (B), 2/7 (A) và 5 (GND) phải được kết nối với bộ chuyển đổi RS485 của ESP8622/ESP32. Có thể sử dụng nhiều bộ chuyển đổi RS485 sang TTL khác nhau.

Trong quá trình thử nghiệm, tôi thấy rằng đầu nối 120 Ohm trong bộ chuyển đổi là không cần thiết. Bộ chuyển đổi USB Seplos V3 gốc cũng không có đầu nối này. Nếu chỉ cần đọc một BMS, cần kết nối chân 6 (B) với chân 5 (GND) để thiết bị chủ có thể gửi dữ liệu độc lập.
![pinout](https://github.com/user-attachments/assets/1c8ec271-d20f-4a5d-baf4-87e5a98fc35a)

Các điểm dữ liệu sau đây hiện đang được đọc:

```
pack_voltage
current
remaining_capacity
total_capacity
total_discharge_capacity
soc
soh
cycle_count
average_cell_voltage
average_cell_temp
max_cell_voltage
min_cell_voltage
max_cell_temp
min_cell_temp
maxdiscurt
maxchgcurt
cell_1_voltage
cell_2_voltage
cell_3_voltage
cell_4_voltage
cell_5_voltage
cell_6_voltage
cell_7_voltage
cell_8_voltage
cell_9_voltage
cell_10_voltage
cell_11_voltage
cell_12_voltage
cell_13_voltage
cell_14_voltage
cell_15_voltage
cell_16_voltage
cell_temp_1
cell_temp_2
cell_temp_3
cell_temp_4
case_temp
power_temp
system_status
active_balancing_cells
cell_temperature_alarms
cell_voltage_alarms
FET_status
active_alarms
active_protections
```

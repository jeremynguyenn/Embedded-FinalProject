MÔN: HỆ THỐNG NHÚNG 
Project cuối kỳ xây dựng hệ thống ứng dụng hoàn chỉnh cho module mà các nhóm đã sử dụng trong bài KT 3

GVHD: TS. Bùi Hà Đức.

Tên SV thực hiện: Nguyễn Trung Nhân - 211460333

Tên SV thực hiện: Trần Trương Huy Hoàng - 21146389

Nhóm tụi em gồm 2 thành viên và nhóm tụi em chọn con cảm biến module cảm biến áp suất nhiet do BMP180 để xây dụng HỆ THỐNG GIÁM SÁT ÁP SUẤT TRONG MÔI TRƯỜNG TRONG CABIN Ô TÔ SỬ DỤNG CẢM BIẾN BMP180

Main system:
```
app.c
```

Flow chart
```
[Start]
   |
   v
[Initialize]
   - Register SIGINT handler
   - Setup WiringPi
   - Initialize PWM for RGB LED (set to WHITE)
   - Open I2C device (/dev/i2c-bmp180)
   - Initialize SPI and MAX7219
   - Initialize variables (pressure/altitude history, start time, sea level pressure)
   - Get BMP180 coefficients and enable debug
   |
   v
[Loop: keep_running == true]
   |
   v
[Read Data]
   - Read temperature (IOCTL_BMP180_GET_TEMP)
   - Read pressure (IOCTL_BMP180_GET_PRES)
   - Get OSS (IOCTL_BMP180_GET_OSS)
   - Add random noise to pressure (±10 Pa)
   |
   v
[Calculate]
   - Compute altitude: 44330 * (1 - (pres / sea_level_pressure)^0.1903)
   - Update sea level pressure (get_sea_level_pressure)
   - Store pressure/altitude in history
   - Compute moving average for pressure/altitude
   - Calculate elapsed time
   |
   v
[Display]
   - Print temperature, pressure, altitude, smoothed values, sea level pressure
   - Display temperature on MAX7219
   |
   v
[Check Previous Data]
   | Yes (previous_pres != 0)
   |---------------------------|
   |                           |
   v                           |
[Warnings & Events]            |
   - Check temperature warning  |
   - Check pressure warning     |
   - Check altitude warning     |
   - Detect pressure change     |
   - Detect pressure trend      |
   - Detect pressure fluctuation|
   - Detect no pressure change  |
   - Detect cabin tampering     |
   - Detect tunnel entry        |
   |                           |
   v                           |
[Adjustments]                  |
   - Compensate air pressure    |
   - Adjust air system (HVAC)   |
   - Optimize fuel/air system   |
   - Adjust air suspension      |
   - Control pressure at high speed
   |                           |
   v                           |
[Log]                         |
   - Log data/events to file    |
   |                           |
   v                           |
[Update]                      |
   - Save current pressure/altitude as previous
   - Read pressure with OSS=3
   - Recalculate altitude
   - Print high-OSS data
   |                           |
   v                           |
[Sleep & Clear]               |
   - Sleep 5 seconds           |
   - Clear console             |
   |                           |
   |---------------------------|
   | No (first reading)
   |---------------------------|
   |                           |
   v                           |
[Print: First reading]         |
   |                           |
   v                           |
[Log]                         |
   - Log data to file          |
   |                           |
   v                           |
[Adjust Suspension]            |
   - Adjust air suspension     |
   |                           |
   v                           |
[Sleep & Clear]               |
   - Sleep 5 seconds           |
   - Clear console             |
   |                           |
   v                           |
[End Loop]--------------------|
   |
   v
[End]
   - Print "Program ended"
   - Close I2C device
   - Exit
```

Result in Max7219 7segment
```
Các trạng thái được hiển thị bao gồm:
Nhiệt độ cabin (số, ví dụ: 25°C).
Thay đổi áp suất đột ngột (ký tự 'S').
Trạng thái HVAC: 'A' (kích hoạt), 'F' (không hoạt động), 'O' (nhiệt độ tối ưu).
Cảnh báo áp suất cao/thấp (ký tự '8' nhấp nháy).
Cảnh báo độ cao (ký tự '6' nhấp nháy với độ sáng thay đổi).
```

## Demo video
https://youtu.be/dVM61QyBQZc?si=pwpl0osD1-c442xa

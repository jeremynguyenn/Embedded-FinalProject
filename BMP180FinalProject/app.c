//Author: Nguyen TRung Nhan (thang nao lay code ko xin phep 1tr - khoi cai vi t danh dau ten t vo trong day roi ko chi moi dau trang dau nen  cai ho =))) )
// Date: 2024-01-01
// Version: 5.1

#include <sys/fcntl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include "bmp180_ioctl.h"
#include <stdlib.h>
#include "MAX7219.h" // Added for MAX7219 functions
#include <wiringPi.h>
#include <softPwm.h>

static volatile bool keep_running = true; // Bien volatile luu trang thai chay chuong trinh

// Ngưỡng cảnh báo
#define PRESSURE_LOW_THRESHOLD       100000  // Pa, nguong ap suat thap de canh bao
#define PRESSURE_HIGH_THRESHOLD      101000  // Pa, nguong ap suat cao de canh bao
#define ALTITUDE_WARN_THRESHOLD      20.0    // m, nguong do cao de canh bao
#define SEA_LEVEL_PRESSURE           101325.0 // Pa, ap suat muc nuoc bien mac dinh
#define PRESSURE_CHANGE_THRESHOLD    10      // Pa, nguong thay doi ap suat de phat hien
#define ALTITUDE_DROP_TUNNEL_ENTRY   2.0     // m, nguong giam do cao de phat hien vao ham
#define MOVING_AVERAGE_WINDOW        5       // So mau cho tinh trung binh truot
#define TAMPERING_THRESHOLD          100     // Pa, nguong phat hien xam nhap cabin

// Cân bằng áp suất khi đóng/mở cửa nhanh
#define RAPID_PRESSURE_CHANGE_THRESHOLD 500 // Pa, nguong thay doi ap suat nhanh
#define COMPENSATION_DURATION 2 // Giay, thoi gian kich hoat he thong can bang

// Điều chỉnh hệ thống treo khí nén theo độ cao
#define ALTITUDE_LOW  0.0   // m, do cao thap
#define ALTITUDE_MED  40.0  // m, do cao trung binh
#define ALTITUDE_HIGH 80.0 // m, do cao cao

// Tự động kiểm soát áp suất khi di chuyển ở tốc độ cao
#define HIGH_SPEED_THRESHOLD 50.0 // km/h, nguong toc do cao
#define TARGET_PRESSURE_MIN 100000 // Pa, ap suat muc tieu toi thieu
#define TARGET_PRESSURE_MAX 101100 // Pa, ap suat muc tieu toi da

// Ngưỡng cải tiến cho các trường hợp
#define TREND_WINDOW 10          // So mau de kiem tra xu huong tang/giam
#define TREND_THRESHOLD 50      // Pa, nguong thay doi ap suat cho xu huong
#define STDEV_THRESHOLD 200     // Pa, nguong do lech chuan cho dao dong bat thuong
#define NO_CHANGE_THRESHOLD 10  // Pa, nguong chenh lech nho coi la khong doi
#define NO_CHANGE_DURATION 300  // Giay, thoi gian ap suat khong doi (5 phut)

// Ngưỡng nhiệt độ
#define TEMP_HIGH_THRESHOLD         30.0 // Nhiet do cao de canh bao
#define TEMP_LOW_THRESHOLD          0.0 // Nhiet do thap de canh bao
#define TEMP_VENTILATION_DURATION   5 // Giay, thoi gian thong gio khi nhiet do cao

// Ngưỡng nhiệt độ cho HVAC
#define HVAC_TEMP_COOL_THRESHOLD    32.0 // Nhiet do bat he thong lam mat
#define HVAC_TEMP_HEAT_THRESHOLD    18.0 // Nhiet do bat he thong suoi
#define HVAC_TEMP_OPTIMAL_MIN       23.0 // Nhiet do toi uu toi thieu
#define HVAC_TEMP_OPTIMAL_MAX       25.0 // Nhiet do toi uu toi da
#define OPTIMAL_STABLE_TIME         0.1 // Giay, thoi gian on dinh cho hysteresis

bool vehicle_is_parked = true; // Bien gia dinh xe dang do

// Định nghĩa chân GPIO cho LED RGB
#define RED_PIN    11  // GPIO 17, chan vat ly 11, chan cho LED do
#define GREEN_PIN  13  // GPIO 27, chan vat ly 13, chan cho LED xanh la
#define BLUE_PIN   15  // GPIO 22, chan vat ly 15, chan cho LED xanh duong

// Định nghĩa cho hiệu ứng sáng/tối dần
#define STEP_DELAY 25    // ms, thoi gian tre cho moi buoc PWM
#define STEPS      100   // So buoc PWM, 100 buoc cho moi chieu (sang/toi)

// Ham fadeUpDown: Tao hieu ung sang toi dan cho LED RGB
void fadeUpDown(int r, int g, int b, int r_target, int g_target, int b_target) {
    // Sang dan tu (r, g, b) den (r_target, g_target, b_target)
    for (int i = 0; i <= STEPS; i++) { // Vong lap qua cac buoc PWM
        softPwmWrite(RED_PIN,   r + (r_target - r) * i / STEPS); // Dieu chinh do sang LED do
        softPwmWrite(GREEN_PIN, g + (g_target - g) * i / STEPS); // Dieu chinh do sang LED xanh la
        softPwmWrite(BLUE_PIN,  b + (b_target - b) * i / STEPS); // Dieu chinh do sang LED xanh duong
        delay(STEP_DELAY); // Cho mot khoang thoi gian giua cac buoc
    }
    // Ghi log su kien
    FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
    if (log) { // Kiem tra file log co mo duoc khong
        fprintf(log, "[RGB EVENT] LED transitioned to R=%d, G=%d, B=%d\n", r_target, g_target, b_target); // Ghi su kien thay doi mau LED
        fclose(log); // Dong file log
    }
}

// Ham xu ly tin hieu de dung chuong trinh
void signal_handler(int sig) { // Nhan tin hieu ngat (SIGINT)
    keep_running = false; // Dat bien keep_running ve false de dung vong lap chinh
}

// Ham tinh trung binh truot
double moving_average(double *values, int size, int window) { // Tinh trung binh truot cho mang gia tri
    if (size < 1) return 0.0; // Neu kich thuoc mang < 1, tra ve 0
    double sum = 0.0; // Bien luu tong gia tri
    int count = size < window ? size : window; // Lay so mau toi da la window hoac size neu size nho hon
    for (int i = size - count; i < size; i++) { // Vong lap qua cac gia tri moi nhat
        sum += values[i]; // Cong don gia tri
    }
    return sum / count; // Tra ve trung binh
}

// Ham tinh do lech chuan
double calculate_stdev(double *values, int size, int window, double mean) { // Tinh do lech chuan cho mang gia tri
    if (size < 1) return 0.0; // Neu kich thuoc mang < 1, tra ve 0
    double sum_squares = 0.0; // Bien luu tong binh phuong chenh lech
    int count = size < window ? size : window; // Lay so mau toi da la window hoac size
    for (int i = size - count; i < size; i++) { // Vong lap qua cac gia tri moi nhat
        sum_squares += pow(values[i] - mean, 2); // Tinh tong binh phuong chenh lech so voi trung binh
    }
    return sqrt(sum_squares / count); // Tra ve can bac hai cua trung binh binh phuong
}

// Ham mo phong lay ap suat tham chieu tu API thoi tiet
// double get_sea_level_pressure() { // Ham ban dau, da duoc toggle comment
//     static double base_pressure = SEA_LEVEL_PRESSURE; // Bien tinh luu ap suat co so
//     base_pressure += (rand() % 100 - 50); // Mo phong dao dong ±50 Pa
//     return base_pressure; // Tra ve ap suat
// }

// Ham mo phong lay ap suat tham chieu voi gioi han pham vi
// double get_sea_level_pressure() { // Ham thu hai, da duoc toggle comment
//     static double base_pressure = SEA_LEVEL_PRESSURE; // Bien tinh luu ap suat co so
//     base_pressure += (rand() % 100 - 50); // Mo phong dao dong ±50 Pa
//     // Gioi han trong pham vi thuc te
//     if (base_pressure < 95000.0) base_pressure = 95000.0; // Gioi han ap suat toi thieu
//     if (base_pressure > 105000.0) base_pressure = 105000.0; // Gioi han ap suat toi da
//     return base_pressure; // Tra ve ap suat
// }

// Ham mo phong lay ap suat tham chieu voi dao dong theo thoi gian
double get_sea_level_pressure() { // Ham hien tai
    static double base_pressure = SEA_LEVEL_PRESSURE; // Bien tinh luu ap suat co so
    time_t current_time; // Bien luu thoi gian hien tai
    time(&current_time); // Lay thoi gian hien tai
    double variation = 100.0 * sin(current_time / 600.0); // Tinh dao dong ±100 Pa voi chu ky 10 phut
    base_pressure = SEA_LEVEL_PRESSURE + variation; // Cap nhat ap suat co so
    if (base_pressure < 95000.0) base_pressure = 95000.0; // Gioi han ap suat toi thieu
    if (base_pressure > 105000.0) base_pressure = 105000.0; // Gioi han ap suat toi da
    return base_pressure; // Tra ve ap suat
}

// Ham kiem tra canh bao ap suat cabin va do cao
// void check_pressure_warning(long pressure, bool skip_leak) { // Ham ban dau, da duoc toggle comment
//     if (pressure < PRESSURE_LOW_THRESHOLD && !skip_leak) { // Kiem tra ap suat thap va khong bo qua ro ri
//         printf(">>> CANH BAO: Ap suat cabin THAP! Co the cua/kinh dang mo hoac bi ro khi.\n"); // In canh bao ap suat thap
//     } else if (pressure > PRESSURE_HIGH_THRESHOLD) { // Kiem tra ap suat cao
//         printf(">>> CANH BAO: Ap suat cabin CAO! Kiem tra he thong thong gio.\n"); // In canh bao ap suat cao
//     } else {
//         printf(">>> Ap suat cabin BINH THUONG.\n"); // In thong bao ap suat binh thuong
//     }
// }

/////////////////Ben trong Cabin xe oto////////////////////

// Ham kiem tra canh bao ap suat cabin va do cao voi LED nhap nhay
void check_pressure_warning(long pressure, bool skip_leak) { // Ham hien tai
    static bool pressure_warning_active = false; // Bien tinh luu trang thai canh bao ap suat
    if (pressure < PRESSURE_LOW_THRESHOLD && !skip_leak) { // Kiem tra ap suat thap va khong bo qua ro ri
        printf(">>> CANH BAO: Ap suat cabin THAP! Co the cua/kinh dang mo hoac bi ro khi.\n"); // In canh bao ap suat thap
        pressure_warning_active = true; // Kich hoat canh bao
    } else if (pressure > PRESSURE_HIGH_THRESHOLD) { // Kiem tra ap suat cao
        printf(">>> CANH BAO: Ap suat cabin CAO! Kiem tra he thong thong gio.\n"); // In canh bao ap suat cao
        pressure_warning_active = true; // Kich hoat canh bao
    } else {
        printf(">>> Ap suat cabin BINH THUONG.\n"); // In thong bao ap suat binh thuong
        pressure_warning_active = false; // Huy canh bao
        sendDataSPI(4, 0x00); // Tat LED tai vi tri digit 4 khi khong co canh bao
    }

    // Xu ly nhap nhay cho canh bao ap suat
    if (pressure_warning_active) { // Neu canh bao dang kich hoat
        static int blink_state = 0; // Bien tinh luu trang thai nhap nhay
        static time_t last_blink = 0; // Bien tinh luu thoi gian nhap nhay truoc do
        time_t current_time; // Bien luu thoi gian hien tai
        time(&current_time); // Lay thoi gian hien tai
        if (difftime(current_time, last_blink) >= 0.1) { // Nhap nhay moi 0.1 giay
            blink_state = !blink_state; // Dao trang thai nhap nhay
            sendDataSPI(4, blink_state ? 0x7F : 0x00); // Hien thi '8' hoac tat LED
            last_blink = current_time; // Cap nhat thoi gian nhap nhay
        }
    }
}

// Ham kiem tra canh bao do cao
// void check_altitude_warning(double altitude) { // Ham ban dau, da duoc toggle comment
//     if (altitude > ALTITUDE_WARN_THRESHOLD) { // Kiem tra do cao vuot nguong
//         printf(">>> CANH BAO: Do cao vuot %.1f m! Co the anh huong den he thong phun nhien lieu.\n", ALTITUDE_WARN_THRESHOLD); // In canh bao do cao
//     }
// }

// Ham kiem tra canh bao do cao voi dieu chinh do sang
// void check_altitude_warning(double altitude) { // Ham thu hai, da duoc toggle comment
//     static int warning = 0; // Bien tinh luu trang thai canh bao do cao
//     static int brightness = 0x00; // Bien tinh luu do sang hien tai
//     static int dir = 1; // Bien tinh luu huong thay doi do sang

//     if (altitude > ALTITUDE_WARN_THRESHOLD) { // Kiem tra do cao vuot nguong
//         printf(">>> CANH BAO: Do cao vuot %.1f m! Co the anh huong den he thong phun nhien lieu.\n", ALTITUDE_WARN_THRESHOLD); // In canh bao do cao
//         warning = 1; // Kich hoat canh bao
//         sendDataSPI(3, 0x7E); // Hien thi pattern '6' tai digit 3
//         sendDataSPI(MAX7219_REG_INTENSITY, brightness); // Cap nhat do sang
//         brightness += dir; // Tang/giam do sang
//         if (brightness >= 0x0F) dir = -1; // Dat do sang toi da, chuyen sang giam
//         else if (brightness <= 0x01) dir = 1; // Dat do sang toi thieu, chuyen sang tang
//     } else {
//         if (warning == 1) { // Neu dang chuyen tu trang thai canh bao sang binh thuong
//             warning = 0; // Huy canh bao
//             sendDataSPI(3, 0x00); // Tat LED tai digit 3
//             sendDataSPI(MAX7219_REG_INTENSITY, 0x08); // Dat lai do sang mac dinh 50%
//             brightness = 0x00; // Dat lai do sang cho lan canh bao tiep theo
//             dir = 1; // Dat lai huong tang do sang
//         }
//     }
// }

// Ham kiem tra canh bao do cao voi dieu chinh do sang va xu huong do cao
void check_altitude_warning(double altitude) { // Ham hien tai
    static int warning = 0; // Bien tinh luu trang thai canh bao do cao
    static int brightness = 0x01; // Bien tinh luu do sang hien tai, bat dau tu toi thieu
    static int dir = 1; // Bien tinh luu huong thay doi do sang
    static int default_brightness = 0x0C; // Bien tinh luu do sang mac dinh (50%)
    static double prev_altitude = 0; // Bien tinh luu do cao truoc do de phat hien xu huong

    // Luu do sang hien tai cua toan bo he thong (neu can)
    static int saved_brightness = 0x0C; // Gia dinh do sang ban dau la 0x0C

    if (altitude > ALTITUDE_WARN_THRESHOLD) { // Kiem tra do cao vuot nguong
        if (warning == 0) { // Neu la lan dau kich hoat canh bao
            // Luu do sang hien tai khi bat dau canh bao
            saved_brightness = default_brightness; // Luu do sang mac dinh
            warning = 1; // Kich hoat canh bao
            brightness = 0x01; // Bat dau tu do sang thap nhat
            dir = 1; // Dat huong tang do sang
        }

        printf(">>> CANH BAO: Do cao vuot %.1f m! Co the anh huong den he thong phun nhien lieu.\n", ALTITUDE_WARN_THRESHOLD); // In canh bao do cao
        sendDataSPI(3, 0x7E); // Hien thi pattern '0' tai digit 3

        // Tang do sang
        sendDataSPI(MAX7219_REG_INTENSITY, brightness); // Cap nhat do sang
        brightness += dir; // Tang/giam do sang
        if (brightness >= 0x0F) dir = -1; // Dat do sang toi da, chuyen sang giam
        else if (brightness <= 0x01) dir = 1; // Dat do sang toi thieu, chuyen sang tang

    } else {
        if (warning == 1) { // Neu dang chuyen tu trang thai canh bao sang binh thuong
            // Do cao giam dan, dieu chinh do sang dua tren xu huong
            if (prev_altitude > altitude) { // Neu do cao dang giam
                // Do cao dang giam, giam do sang
                brightness -= 1; // Giam do sang
                if (brightness < 0x01) brightness = 0x01; // Gioi han do sang toi thieu
                sendDataSPI(MAX7219_REG_INTENSITY, brightness); // Cap nhat do sang
                sendDataSPI(3, 0x7E); // Van hien thi '6' khi do sang giam
            }

            // Neu do cao duoi nguong, tat canh bao
            if (altitude <= ALTITUDE_WARN_THRESHOLD) { // Kiem tra do cao duoi nguong
                warning = 0; // Huy canh bao
                sendDataSPI(3, 0x00); // Tat LED tai digit 3
                sendDataSPI(MAX7219_REG_INTENSITY, saved_brightness); // Khoi phuc do sang mac dinh
                brightness = 0x01; // Dat lai do sang cho lan canh bao tiep theo
                dir = 1; // Dat lai huong tang do sang
            }
        }
    }

    // Cap nhat do cao truoc do
    prev_altitude = altitude; // Luu do cao hien tai de so sanh lan sau
}

// Ham kiem tra canh bao nhiet do
void check_temperature_warning(int device, double tempC, bool parked) { // Ham kiem tra canh bao nhiet do
    static bool temp_warning_active = false; // Bien tinh luu trang thai canh bao nhiet do

    if (parked && tempC > TEMP_HIGH_THRESHOLD) { // Neu xe dang do va nhiet do cao vuot nguong
        printf(">>> CANH BAO: Nhiet do cabin QUA CAO (%.1f°C)! Nguy hiem cho hanh khach/thu cung.\n", tempC); // In canh bao
        temp_warning_active = true; // Kich hoat canh bao

        int res = ioctl(device, IOCTL_BMP180_ACTIVATE_VENTILATION, NULL); // Kich hoat he thong thong gio
        if (res < 0) { // Kiem tra loi
            perror("Khong the kich hoat he thong thong gio"); // In thong bao loi
        } else {
            printf(">>> He thong thong gio dang chay trong %d giay...\n", TEMP_VENTILATION_DURATION); // In thong bao
            sleep(TEMP_VENTILATION_DURATION); // Cho he thong chay trong thoi gian quy dinh
            res = ioctl(device, IOCTL_BMP180_DEACTIVATE_VENTILATION, NULL); // Tat he thong thong gio
            if (res < 0) { // Kiem tra loi
                perror("Khong the tat he thong thong gio"); // In thong bao loi
            } else {
                printf(">>> He thong thong gio da tat.\n"); // In thong bao thanh cong
            }
        }

        FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
        if (log) { // Kiem tra file log co mo duoc khong
            fprintf(log, "[CANH BAO] Nhiet do cabin qua cao: %.1f°C, kich hoat thong gio\n", tempC); // Ghi su kien
            fclose(log); // Dong file log
        }
    } else if (parked && tempC < TEMP_LOW_THRESHOLD) { // Neu xe dang do va nhiet do thap duoi nguong
        printf(">>> CANH BAO: Nhiet do cabin QUA THAP (%.1f°C)! Kiem tra he thong suoi.\n", tempC); // In canh bao
        temp_warning_active = true; // Kich hoat canh bao

        FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
        if (log) { // Kiem tra file log co mo duoc khong
            fprintf(log, "[CANH BAO] Nhiet do cabin qua thap: %.1f°C\n", tempC); // Ghi su kien
            fclose(log); // Dong file log
        }
    } else {
        if (temp_warning_active) { // Neu dang chuyen tu trang thai canh bao sang binh thuong
            printf(">>> Nhiet do cabin BINH THUONG (%.1f°C).\n", tempC); // In thong bao
            temp_warning_active = false; // Huy canh bao

            FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
            if (log) { // Kiem tra file log co mo duoc khong
                fprintf(log, "[SỰ KIỆN] Nhiet do cabin tro ve binh thuong: %.1f°C\n", tempC); // Ghi su kien
                fclose(log); // Dong file log
            }
        }
    }
}

// Ham phat hien thay doi ap suat dot ngot
// void detect_pressure_change(long prev, long curr) { // Ham ban dau, da duoc toggle comment
//     long diff = curr - prev; // Tinh chenh lech ap suat
//     if (abs(diff) > PRESSURE_CHANGE_THRESHOLD) { // Kiem tra neu chenh lech vuot nguong
//         printf(">>> CANH BAO: Ap suat cabin thay doi dot ngot: %ld Pa.\n", diff); // In canh bao
//     }
// }

// Ham phat hien thay doi ap suat dot ngot voi hien thi LED
void detect_pressure_change(long prev, long curr) { // Ham hien tai
    long diff = curr - prev; // Tinh chenh lech ap suat
    if (abs(diff) > PRESSURE_CHANGE_THRESHOLD) { // Kiem tra neu chenh lech vuot nguong
        printf(">>> CANH BAO: Ap suat cabin thay doi dot ngot: %ld Pa.\n", diff); // In canh bao
        sendDataSPI(1, 0x5D); // Hien thi 'S' khi phat hien thay doi ap suat dot ngot
        // Ghi log su kien
        FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
        if (log) { // Kiem tra file log co mo duoc khong
            fprintf(log, "[SỰ KIỆN] Thay đổi áp suất đột ngột: %ld Pa, hiển thị 'S' trên digit 1\n", diff); // Ghi su kien
            fclose(log); // Dong hindering file log
        }
    } else {
        sendDataSPI(1, 0x01); // Hien thi '-' (dau ngang tai doan g) cho trang thai binh thuong
        // Ghi log su kien
        FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
        if (log) { // Kiem tra file log co mo duoc khong
            fprintf(log, "[SỰ KIỆN] Áp suất bình thường, hiển thị '-' trên digit 1\n"); // Ghi su kien
            fclose(log); // Dong file log
        }
    }
}


// Ham phat hien xu huong ap suat (tang hoac giam lien tuc)
void detect_pressure_trend(double *pressure_history, int size, int window) { // Ham kiem tra xu huong ap suat
    if (size < window) return; // Neu so mau khong du, thoat ham
    bool increasing = true, decreasing = true; // Bien kiem tra xu huong tang/giam
    double total_change = 0.0; // Bien luu tong thay doi ap suat
    for (int i = size - window; i < size - 1; i++) { // Vong lap qua cac mau trong cua so
        double diff = pressure_history[i + 1] - pressure_history[i]; // Tinh chenh lech giua hai mau lien tiep
        total_change += diff; // Cong don chenh lech
        if (diff <= -TREND_THRESHOLD) increasing = false; // Neu giam qua nguong, khong phai xu huong tang
        if (diff >= TREND_THRESHOLD) decreasing = false; // Neu tang qua nguong, khong phai xu huong giam
    }
    if (decreasing && abs(total_change) > TREND_THRESHOLD) { // Neu xu huong giam va tong thay doi vuot nguong
        printf(">>> CANH BAO: Ap suat cabin GIAM DAN LIEN TUC: %.1f Pa qua %d mau.\n", total_change, window); // In canh bao
        printf(">>> HANH DONG: Tang toc quat de bu khi.\n"); // De xuat hanh dong
    } else if (increasing && abs(total_change) > TREND_THRESHOLD) { // Neu xu huong tang va tong thay doi vuot nguong
        printf(">>> CANH BAO: Ap suat cabin TANG DAN LIEN TUC: %.1f Pa qua %d mau.\n", total_change, window); // In canh bao
        printf(">>> HANH DONG: Giam toc quat, mo van tuan hoan.\n"); // De xuat hanh dong
    }
}

// Ham phat hien dao dong bat thuong
void detect_pressure_fluctuation(double *pressure_history, int size, int window, double mean) { // Ham kiem tra dao dong ap suat
    double stdev = calculate_stdev(pressure_history, size, window, mean); // Tinh do lech chuan
    if (stdev > STDEV_THRESHOLD) { // Neu do lech chuan vuot nguong
        printf(">>> CANH BAO: Ap suat dao dong bat thuong! Do lech chuan: %.1f Pa.\n", stdev); // In canh bao
        printf(">>> HANH DONG: Bo qua ket qua, kiem tra cam bien BMP180.\n"); // De xuat hanh dong
    }
}

// Ham phat hien ap suat khong doi trong thoi gian dai
void detect_no_pressure_change(double *pressure_history, int size, int window, double duration, bool *skip_leak) { // Ham kiem tra ap suat khong doi
    if (size < window) return; // Neu so mau khong du, thoat ham
    bool no_change = true; // Bien kiem tra ap suat co khong doi
    for (int i = size - window; i < size - 1; i++) { // Vong lap qua cac mau trong cua so
        if (abs(pressure_history[i + 1] - pressure_history[i]) > NO_CHANGE_THRESHOLD) { // Kiem tra chenh lech vuot nguong
            no_change = false; // Dat bien no_change ve false neu co thay doi
            break; // Thoat vong lap
        }
    }
    if (no_change && duration > NO_CHANGE_DURATION) { // Neu ap suat khong doi va thoi gian vuot nguong
        printf(">>> CANH BAO: Ap suat khong thay doi trong %.1f giay! Co the cam bien hong hoac cabin qua kin.\n", duration); // In canh bao
        printf(">>> HANH DONG: Kiem tra cam bien hoac xac nhan cabin kin.\n"); // De xuat hanh dong
        *skip_leak = true; // Bo qua canh bao ro ri neu cabin qua kin
    } else {
        *skip_leak = false; // Cho phep canh bao ro ri
    }
}

// Ham phat hien xam nhap khi do xe
void detect_cabin_tampering(long prev_pres, long curr_pres, bool parked) { // Ham kiem tra xam nhap cabin
    long diff = curr_pres - prev_pres; // Tinh chenh lech ap suat
    if (parked && abs(diff) > TAMPERING_THRESHOLD) { // Neu xe dang do va chenh lech ap suat vuot nguong
        printf(">>> CANH BAO XAM NHAP: Ap suat cabin thay doi %ld Pa khi xe dang do!\n", diff); // In canh bao
    }
}

// Ham can bang ap suat khi dong/mo cua nhanh
void compensate_air_pressure(int device, long pressure_change, bool *skip_leak) { // Ham can bang ap suat
    if (abs(pressure_change) > RAPID_PRESSURE_CHANGE_THRESHOLD) { // Neu chenh lech ap suat vuot nguong nhanh
        printf(">>> CẢNH BÁO: Thay đổi áp suất nhanh (%ld Pa)! Kích hoạt hệ thống cân bằng áp suất.\n", pressure_change); // In canh bao
        *skip_leak = true; // Bo qua canh bao ro ri khi ap suat giam nhanh
        int res = ioctl(device, IOCTL_BMP180_ACTIVATE_VENTILATION, NULL); // Kich hoat he thong can bang ap suat
        if (res < 0) { // Kiem tra loi
            perror("Không thể kích hoạt hệ thống cân bằng áp suất"); // In thong bao loi
        } else {
            printf(">>> Hệ thống cân bằng áp suất đang chạy trong %d giây...\n", COMPENSATION_DURATION); // In thong bao
            sleep(COMPENSATION_DURATION); // Cho he thong chay trong thoi gian quy dinh
            res = ioctl(device, IOCTL_BMP180_DEACTIVATE_VENTILATION, NULL); // Tat he thong can bang ap suat
            if (res < 0) { // Kiem tra loi
                perror("Không thể tắt hệ thống cân bằng áp suất"); // In thong bao loi
            } else {
                printf(">>> Hệ thống cân bằng áp suất đã tắt.\n"); // In thong bao thanh cong
            }
        }
        FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
        if (log) { // Kiem tra file log co mo duoc khong
            fprintf(log, "[SỰ KIỆN] Phát hiện thay đổi áp suất nhanh: %ld Pa, đã kích hoạt cân bằng\n", pressure_change); // Ghi su kien
            fclose(log); // Dong file log
        }
    }
}

//Author: Nguyen TRung Nhan (thang nao lay code ko xin phep 1tr - khoi cai vi t danh dau ten t vo trong day roi ko chi moi dau trang dau nen  cai ho =))) )
// Date: 2024-01-01
// Version: 1.0

// Dieu chinh he thong dieu hoa/loc khi (phien ban 1)
// void auto_adjust_air_system(long pressure_diff) { // Ham ban dau, da duoc toggle comment
//     if (abs(pressure_diff) > PRESSURE_CHANGE_THRESHOLD) { // Kiem tra chenh lech ap suat vuot nguong
//         printf(">>> DIEU CHINH: Kich hoat he thong dieu hoa/loc khi do chenh lech ap suat %ld Pa.\n", pressure_diff); // In thong bao kich hoat
//     }
// }

// Dieu chinh he thong dieu hoa/loc khi voi hien thi LED (phien ban 2)
// void auto_adjust_air_system(long pressure_diff) { // Ham thu hai, da duoc toggle comment
//     if (abs(pressure_diff) > PRESSURE_CHANGE_THRESHOLD) { // Kiem tra chenh lech ap suat vuot nguong
//         printf(">>> DIEU CHINH: Kich hoat he thong dieu hoa/loc khi do chenh lech ap suat %ld Pa.\n", pressure_diff); // In thong bao kich hoat
//         sendDataSPI(2, 0x77); // Hien thi ky tu 'A' tren digit 2 (kich hoat)
//     } else {
//         sendDataSPI(2, 0x47); // Hien thi ky tu 'F' tren digit 2 (khong kich hoat)
//     }
// }

// Ham dieu chinh he thong dieu hoa/loc khi voi IOCTL (phien ban 3)
// void auto_adjust_air_system(int device, long pressure_diff, double tempC) { // Ham thu ba, da duoc toggle comment
//     bool adjusted = false; // Bien kiem tra co dieu chinh hay khong
// 
//     if (abs(pressure_diff) > PRESSURE_CHANGE_THRESHOLD) { // Kiem tra chenh lech ap suat vuot nguong
//         printf(">>> DIEU CHINH: Kich hoat he thong loc khi do chenh lech ap suat %ld Pa.\n", pressure_diff); // In thong bao kich hoat loc khi
//         int res = ioctl(device, IOCTL_BMP180_ACTIVATE_VENTILATION, NULL); // Goi IOCTL de kich hoat he thong loc khi
//         if (res < 0) { // Kiem tra loi
//             perror("Khong the kich hoat he thong loc khi"); // In thong bao loi
//         } else {
//             printf(">>> He thong loc khi dang chay...\n"); // In thong bao he thong dang chay
//         }
//         adjusted = true; // Danh dau da dieu chinh
// 
//         FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
//         if (log) { // Kiem tra file log co mo duoc khong
//             fprintf(log, "[SỰ KIỆN] Kich hoat loc khi do chenh lech ap suat: %ld Pa\n", pressure_diff); // Ghi su kien vao log
//             fclose(log); // Dong file log
//         }
//     }
// 
//     if (tempC > HVAC_TEMP_COOL_THRESHOLD) { // Kiem tra nhiet do cao vuot nguong
//         printf(">>> DIEU CHINH: Bat he thong lam mat do nhiet do cao (%.1f°C).\n", tempC); // In thong bao kich hoat lam mat
//         int res = ioctl(device, IOCTL_BMP180_ACTIVATE_COOLING, NULL); // Goi IOCTL de kich hoat he thong lam mat
//         if (res < 0) { // Kiem tra loi
//             perror("Khong the kich hoat he thong lam mat"); // In thong bao loi
//         } else {
//             printf(">>> He thong lam mat dang chay...\n"); // In thong bao he thong dang chay
//         }
//         adjusted = true; // Danh dau da dieu chinh
// 
//         FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
//         if (log) { // Kiem tra file log co mo duoc khong
//             fprintf(log, "[SỰ KIỆN] Kich hoat lam mat do nhiet do: %.1f°C\n", tempC); // Ghi su kien vao log
//             fclose(log); // Dong file log
//         }
//     } else if (tempC < HVAC_TEMP_HEAT_THRESHOLD) { // Kiem tra nhiet do thap duoi nguong
//         printf(">>> DIEU CHINH: Bat he thong suoi do nhiet do thap (%.1f°C).\n", tempC); // In thong bao kich hoat suoi
//         int res = ioctl(device, IOCTL_BMP180_ACTIVATE_HEATING, NULL); // Goi IOCTL de kich hoat he thong suoi
//         if (res < 0) { // Kiem tra loi
//             perror("Khong the kich hoat he thong suoi"); // In thong bao loi
//         } else {
//             printf(">>> He thong suoi dang chay...\n"); // In thong bao he thong dang chay
//         }
//         adjusted = true; // Danh dau da dieu chinh
// 
//         FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
//         if (log) { // Kiem tra file log co mo duoc khong
//             fprintf(log, "[SỰ KIỆN] Kich hoat suoi do nhiet do: %.1f°C\n", tempC); // Ghi su kien vao log
//             fclose(log); // Dong file log
//         }
//     } else if (tempC >= HVAC_TEMP_OPTIMAL_MIN && tempC <= HVAC_TEMP_OPTIMAL_MAX) { // Kiem tra nhiet do trong khoang toi uu
//         printf(">>> Nhiet do cabin toi uu (%.1f°C), tat he thong dieu hoa.\n", tempC); // In thong bao tat dieu hoa
//         int res = ioctl(device, IOCTL_BMP180_DEACTIVATE_HVAC, NULL); // Goi IOCTL de tat he thong dieu hoa
//         if (res < 0) { // Kiem tra loi
//             perror("Khong the tat he thong dieu hoa"); // In thong bao loi
//         } else {
//             printf(">>> He thong dieu hoa da tat.\n"); // In thong bao he thong da tat
//         }
//         adjusted = true; // Danh dau da dieu chinh
// 
//         FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
//         if (log) { // Kiem tra file log co mo duoc khong
//             fprintf(log, "[SỰ KIỆN] Tat dieu hoa do nhiet do toi uu: %.1f°C\n", tempC); // Ghi su kien vao log
//             VesionId="58699893-641e-4e79-b3e8-0848241707cd" title="app.c" contentType="text/c">
//
//             fclose(log); // Dong file log
//         }
//     }
//     // Hien thi trang thai dieu chinh tren LED
//     if (adjusted) { // Neu co dieu chinh
//         sendDataSPI(2, 0x77); // Hien thi 'A' tren digit 2
//     } else {
//         sendDataSPI(2, 0x47); // Hien thi 'F' tren digit 2
//     }
// }

// Ham dieu chinh he thong dieu hoa/loc khi voi hysteresis
void auto_adjust_air_system(int device, long pressure_diff, double tempC) { // Ham hien tai
    bool adjusted = false; // Bien kiem tra co dieu chinh hay khong
    static double last_optimal_time = 0; // Bien tinh luu thoi gian lan cuoi nhiet do toi uu
    static bool in_optimal_range = false; // Bien tinh luu trang thai nhiet do toi uu
    double current_time = (double)clock() / CLOCKS_PER_SEC; // Lay thoi gian hien tai bang clock

    // Mo file log mot lan
    FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
    if (!log) { // Kiem tra file log co mo duoc khong
        perror("Khong the mo file cabin_log.txt"); // In thong bao loi
    }

    // Kiem tra chenh lech ap suat
    if (abs(pressure_diff) > PRESSURE_CHANGE_THRESHOLD) { // Neu chenh lech ap suat vuot nguong
        printf(">>> DIEU CHINH: Kich hoat he thong loc khi do chenh lech ap suat %ld Pa.\n", pressure_diff); // In thong bao
        int res = ioctl(device, IOCTL_BMP180_ACTIVATE_VENTILATION, NULL); // Kich hoat he thong loc khi
        if (res < 0) { // Kiem tra loi
            perror("Khong the kich hoat he thong loc khi"); // In thong bao loi
        } else {
            printf(">>> He thong loc khi dang chay...\n"); // In thong bao thanh cong
        }
        adjusted = true; // Danh dau da dieu chinh
        if (log) { // Kiem tra file log co mo duoc khong
            fprintf(log, "[SỰ KIỆN] Kich hoat loc khi do chenh lech ap suat: %ld Pa\n", pressure_diff); // Ghi su kien
        }
    }

    // Kiem tra nhiet do
    if (tempC > HVAC_TEMP_COOL_THRESHOLD) { // Neu nhiet do cao vuot nguong
        printf(">>> DIEU CHINH: Bat he thong lam mat do nhiet do cao (%.1f°C).\n", tempC); // In thong bao
        int res = ioctl(device, IOCTL_BMP180_ACTIVATE_COOLING, NULL); // Kich hoat he thong lam mat
        if (res < 0) { // Kiem tra loi
            perror("Khong the kich hoat he thong lam mat"); // In thong bao loi
        } else {
            printf(">>> He thong lam mat dang chay...\n"); // In thong bao thanh cong
        }
        adjusted = true; // Danh dau da dieu chinh
        in_optimal_range = false; // Dat lai trang thai toi uu
        if (log) { // Kiem tra file log co mo duoc khong
            fprintf(log, "[SỰ KIỆN] Kich hoat lam mat do nhiet do: %.1f°C\n", tempC); // Ghi su kien
        }
    } else if (tempC < HVAC_TEMP_HEAT_THRESHOLD) { // Neu nhiet do thap duoi nguong
        printf(">>> DIEU CHINH: Bat he thong suoi do nhiet do thap (%.1f°C).\n", tempC); // In thong bao
        int res = ioctl(device, IOCTL_BMP180_ACTIVATE_HEATING, NULL); // Kich hoat he thong suoi
        if (res < 0) { // Kiem tra loi
            perror("Khong the kich hoat he thong suoi"); // In thong bao loi
        } else {
            printf(">>> He thong suoi dang chay...\n"); // In thong bao thanh cong
        }
        adjusted = true; // Danh dau da dieu chinh
        in_optimal_range = false; // Dat lai trang thai toi uu
        if (log) { // Kiem tra file log co mo duoc khong
            fprintf(log, "[SỰ KIỆN] Kich hoat suoi do nhiet do: %.1f°C\n", tempC); // Ghi su kien
        }
    } else if (tempC >= HVAC_TEMP_OPTIMAL_MIN && tempC <= HVAC_TEMP_OPTIMAL_MAX) { // Neu nhiet do trong khoang toi uu
        // Kiem tra hysteresis: chi tat dieu hoa neu nhiet do on dinh trong khoang toi uu
        if (!in_optimal_range) { // Neu chua o trang thai toi uu
            in_optimal_range = true; // Dat trang thai toi uu
            last_optimal_time = current_time; // Luu thoi gian bat dau toi uu
        }
        if (in_optimal_range && (current_time - last_optimal_time >= OPTIMAL_STABLE_TIME)) { // Neu nhiet do on dinh du thoi gian
            printf(">>> Nhiet do cabin toi uu (%.1f°C), tat he thong dieu hoa.\n", tempC); // In thong bao
            int res = ioctl(device, IOCTL_BMP180_DEACTIVATE_HVAC, NULL); // Tat he thong dieu hoa
            if (res < 0) { // Kiem tra loi
                perror("Khong the tat he thong dieu hoa"); // In thong bao loi
            } else {
                printf(">>> He thong dieu hoa da tat.\n"); // In thong bao thanh cong
            }
            adjusted = true; // Danh dau da dieu chinh
            if (log) { // Kiem tra file log co mo duoc khong
                fprintf(log, "[SỰ KIỆN] Tat dieu hoa do nhiet do toi uu: %.1f°C\n", tempC); // Ghi su kien
            }
        }
    } else {
        in_optimal_range = false; // Dat lai trang thai neu nhiet do ra khoi khoang toi uu
    }

    // Dong file log
    if (log) { // Kiem tra file log co mo duoc khong
        fclose(log); // Dong file log
    }

    // Hien thi trang thai
    if (adjusted) { // Neu co dieu chinh
        if (tempC >= HVAC_TEMP_OPTIMAL_MIN && tempC <= HVAC_TEMP_OPTIMAL_MAX &&
            in_optimal_range && (current_time - last_optimal_time >= OPTIMAL_STABLE_TIME)) { // Neu nhiet do toi uu va on dinh
            sendDataSPI(2, 0x7E); // Hien thi 'O' cho trang thai toi uu
        } else {
            sendDataSPI(2, 0x77); // Hien thi 'A' cho cac dieu chinh khac
        }
    } else {
        sendDataSPI(2, 0x47); // Hien thi 'F' neu khong co dieu chinh
    }
}

// Ham phat hien xe co the dang vao ham
// void detect_tunnel_entry(double prev_alti, double curr_alti, long prev_pres, long curr_pres) { // Ham ban dau, da duoc toggle comment
//     double alti_drop = prev_alti - curr_alti; // Tinh do giam do cao
//     long pres_diff = curr_pres - prev_pres; // Tinh chenh lech ap suat
//     if (alti_drop > ALTITUDE_DROP_TUNNEL_ENTRY && pres_diff > PRESSURE_CHANGE_THRESHOLD) { // Kiem tra dieu kien vao ham
//         printf(">>> PHAT HIEN: XE CO THE DANG VAO HAM! Giam do cao %.1f m, tang ap suat %ld Pa.\n", alti_drop, pres_diff); // In thong bao
//     }
// }

// Ham phat hien vao/ra ham voi hieu ung LED
void detect_tunnel_entry(double prev_alti, double curr_alti, long prev_pres, long curr_pres) { // Ham hien tai
    static bool in_tunnel = false; // Bien tinh luu trang thai trong ham
    static int current_r = 100, current_g = 100, current_b = 100; // Bien tinh luu mau LED hien tai (mac dinh WHITE)

    double alti_diff = prev_alti - curr_alti; // Tinh chenh lech do cao
    long pres_diff = curr_pres - prev_pres; // Tinh chenh lech ap suat

    // Phat hien vao ham: do cao giam va ap suat tang
    if (alti_diff > ALTITUDE_DROP_TUNNEL_ENTRY && pres_diff > PRESSURE_CHANGE_THRESHOLD && !in_tunnel) { // Kiem tra dieu kien vao ham
        printf(">>> PHAT HIEN: XE DANG VAO HAM! Giam do cao %.1f m, tang ap suat %ld Pa.\n", alti_diff, pres_diff); // In thong bao
        in_tunnel = true; // Dat trang thai dang trong ham
        // Chuyen tu WHITE (100, 100, 100) sang Yellow (100, 100, 0) hoac Blue(0,0,100)
        fadeUpDown(current_r, current_g, current_b, 0, 0, 100); // Chuyen mau LED sang Blue
        current_r = 0; // Cap nhat mau do
        current_g = 0; // Cap nhat mau xanh la
        current_b = 100; // Cap nhat mau xanh duong
        // Ghi log su kien
        FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
        if (log) { // Kiem tra file log co mo duoc khong
            fprintf(log, "[SỰ KIỆN] Xe vào hầm, LED chuyển sang Yellow\n"); // Ghi su kien
            fclose(log); // Dong file log
        }
    }
    // Phat hien ra ham: do cao tang va ap suat giam
    else if (alti_diff < -ALTITUDE_DROP_TUNNEL_ENTRY && pres_diff < -PRESSURE_CHANGE_THRESHOLD && in_tunnel) { // Kiem tra dieu kien ra ham
        printf(">>> PHAT HIEN: XE RA KHOI HAM! Tang do cao %.1f m, giam ap suat %ld Pa.\n", -alti_diff, -pres_diff); // In thong bao
        in_tunnel = false; // Dat trang thai khong trong ham
        // Chuyen tu Yellow (100, 100, 0) hoac Blue(0,0,100) sang WHITE (100, 100, 100)
        fadeUpDown(current_r, current_g, current_b, 100, 100, 100); // Chuyen mau LED sang WHITE
        current_r = 100; // Cap nhat mau do
        current_g = 100; // Cap nhat mau xanh la
        current_b = 100; // Cap nhat mau xanh duong
        // Ghi log su kien
        FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
        if (log) { // Kiem tra file log co mo duoc khong
            fprintf(log, "[SỰ KIỆN] Xe ra khỏi hầm, LED chuyển sang WHITE\n"); // Ghi su kien
            fclose(log); // Dong file log
        }
    }
}

/////////////// Dau xe va phan khhung xe///////////////////////////////

// Ham phat hien vao/ra ham voi mo phong du lieu
// void detect_tunnel_entry(double prev_alti, double curr_alti, long prev_pres, long curr_pres) { // Ham da duoc toggle comment
//     static bool in_tunnel = false; // Bien tinh luu trang thai trong ham
//     static int current_r = 100, current_g = 100, current_b = 100; // Bien tinh luu mau LED hien tai (mac dinh WHITE)
//     static int call_counter = 0; // Bien tinh dem so lan goi ham de xen ke vao/ra ham
// 
//     double alti_diff; // Bien luu chenh lech do cao
//     long pres_diff; // Bien luu chenh lech ap suat
// 
//     // Gan gia tri co dinh xen ke cho vao/ra ham
//     if (call_counter % 2 == 0) { // Neu la lan goi chan (mo phong vao ham)
//         // Gia lap vao ham
//         alti_diff = 3.0; // Do cao giam 3.0m (> ALTITUDE_DROP_TUNNEL_ENTRY = 2.0)
//         pres_diff = 101; // Ap suat tang 101 Pa (> PRESSURE_CHANGE_THRESHOLD = 1)
//     } else {
//         // Gia lap ra ham
//         alti_diff = -3.0; // Do cao tang 3.0m
//         pres_diff = -101; // Ap suat giam 101 Pa
//     }
//     call_counter++; // Tang bien dem goi
// 
//     // Phat hien vao ham: do cao giam va ap suat tang
//     if (alti_diff > ALTITUDE_DROP_TUNNEL_ENTRY && pres_diff > PRESSURE_CHANGE_THRESHOLD && !in_tunnel) { // Kiem tra dieu kien vao ham
//         printf(">>> PHAT HIEN: XE DANG VAO HAM! Giam do cao %.1f m, tang ap suat %ld Pa.\n", alti_diff, pres_diff); // In thong bao
//         in_tunnel = true; // Dat trang thai dang trong ham
//         // Chuyen tu WHITE (100, 100, 100) sang Yellow (100, 100, 0) hoac Blue(0,0,100)
//         fadeUpDown(current_r, current_g, current_b, 100, 0, 0); // Chuyen mau LED sang Yellow
//         current_r = 100; // Cap nhat mau do
//         current_g = 0; // Cap nhat mau xanh la
//         current_b = 0; // Cap nhat mau xanh duong
//         // Ghi log su kien
//         FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
//         if (log) { // Kiem tra file log co mo duoc khong
//             fprintf(log, "[SỰ KIỆN] Xe vào hầm, LED chuyển sang Yellow (alti_diff=%.1f, pres_diff=%ld)\n", alti_diff, pres_diff); // Ghi su kien
//             fclose(log); // Dong file log
//         }
//     }
//     // Phat hien ra ham: do cao tang va ap suat giam
//     else if (alti_diff < -ALTITUDE_DROP_TUNNEL_ENTRY && pres_diff < -PRESSURE_CHANGE_THRESHOLD && in_tunnel) { // Kiem tra dieu kien ra ham
//         printf(">>> PHAT HIEN: XE RA KHOI HAM! Tang do cao %.1f m, giam ap suat %ld Pa.\n", -alti_diff, -pres_diff); // In thong bao
//         in_tunnel = false; // Dat trang thai khong trong ham
//         // Chuyen tu Yellow (100, 100, 0) hoac Blue(0,0,100) sang WHITE (100, 100, 100)
//         fadeUpDown(current_r, current_g, current_b, 100, 100, 100); // Chuyen mau LED sang WHITE
//         current_r = 100; // Cap nhat mau do
//         current_g = 100; // Cap nhat mau xanh la
//         current_b = 100; // Cap nhat mau xanh duong
//         // Ghi log su kien
//         FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
//         if (log) { // Kiem tra file log co mo duoc khong
//             fprintf(log, "[SỰ KIỆN] Xe ra khỏi hầm, LED chuyển sang WHITE (alti_diff=%.1f, pres_diff=%ld)\n", alti_diff, pres_diff); // Ghi su kien
//             fclose(log); // Dong file log
//         }
//     }
// }


//Author: Nguyen TRung Nhan
// Date: 2024-01-01
// Version: 1.0

// Ham toi uu phun nhien lieu/dieu hoa
void optimize_fuel_and_air_system(double altitude) { // Ham toi uu he thong phun nhien lieu va dieu hoa
    if (altitude > 15.0) { // Neu do cao vuot 15m
        printf(">>> TOI UU: Dieu chinh he thong phun nhien lieu/loc khi (do cao > 15m).\n"); // In thong bao
    } else {
        printf(">>> TOI UU: Van hanh binh thuong (do cao < 15m).\n"); // In thong bao
    }
}

// Ham dieu chinh he thong treo khi nen theo do cao
void adjust_air_suspension(int device, double altitude) { // Ham dieu chinh he thong treo khi nen
    int suspension_level; // Bien luu muc do treo
    if (altitude < ALTITUDE_LOW) { // Neu do cao duoi nguong thap
        suspension_level = 1; // Dat muc cung
        printf(">>> Điều chỉnh hệ thống treo khí nén: Mức CỨNG (độ cao %.1f m).\n", altitude); // In thong bao
    } else if (altitude < ALTITUDE_MED) { // Neu do cao duoi nguong trung binh
        suspension_level = 2; // Dat muc trung binh
        printf(">>> Điều chỉnh hệ thống treo khí nén: Mức TRUNG BÌNH (độ cao %.1f m).\n", altitude); // In thong bao
    } else {
        suspension_level = 3; // Dat muc mem
        printf(">>> Điều chỉnh hệ thống treo khí nén: Mức MỀM (độ cao %.1f m).\n", altitude); // In thong bao
    }
    int res = ioctl(device, IOCTL_BMP180_SET_SUSPENSION, &suspension_level); // Dieu chinh he thong treo
    if (res < 0) { // Kiem tra loi
        perror("Không thể điều chỉnh hệ thống treo khí nén"); // In thong bao loi
    }
    FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
    if (log) { // Kiem tra file log co mo duoc khong
        fprintf(log, "[SỰ KIỆN] Hệ thống treo khí nén được điều chỉnh đến mức %d tại độ cao %.1f m\n", suspension_level, altitude); // Ghi su kien
        fclose(log); // Dong file log
    }
}

// Ham tu dong kiem soat ap suat khi di chuyen o toc do cao
void control_pressure_at_high_speed(int device, double smoothed_pressure, double speed) { // Ham kiem soat ap suat tai toc do cao
    if (speed > HIGH_SPEED_THRESHOLD) { // Neu toc do vuot nguong
        printf(">>> Tốc độ cao (%.1f km/h)! Kiểm tra và điều chỉnh áp suất cabin.\n", speed); // In thong bao
        if (smoothed_pressure < TARGET_PRESSURE_MIN) { // Neu ap suat thap hon muc toi thieu
            printf(">>> Áp suất thấp (%.1f Pa), tăng áp suất cabin.\n", smoothed_pressure); // In thong bao
            int res = ioctl(device, IOCTL_BMP180_INCREASE_PRESSURE, NULL); // Tang ap suat
            if (res < 0) { // Kiem tra loi
                perror("Không thể tăng áp suất cabin"); // In thong bao loi
            }
        } else if (smoothed_pressure > TARGET_PRESSURE_MAX) { // Neu ap suat cao hon muc toi da
            printf(">>> Áp suất cao (%.1f Pa), giảm áp suất cabin.\n", smoothed_pressure); // In thong bao
            int res = ioctl(device, IOCTL_BMP180_DECREASE_PRESSURE, NULL); // Giam ap suat
            if (res < 0) { // Kiem tra loi
                perror("Không thể giảm áp suất cabin"); // In thong bao loi
            }
        } else {
            printf(">>> Áp suất cabin ở mức tối ưu (%.1f Pa) tại tốc độ cao.\n", smoothed_pressure); // In thong bao
        }
        FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
        if (log) { // Kiem tra file log co mo duoc khong
            fprintf(log, "[SỰ KIỆN] Kiểm soát áp suất tốc độ cao: tốc độ %.1f km/h, áp suất %.1f Pa\n", speed, smoothed_pressure); // Ghi su kien
            fclose(log); // Dong file log
        }
    }
}


// Ham ghi log du lieu vao file
void log_to_file(double tempC, long pressure, double altitude, double duration) { // Ham ghi log du lieu
    FILE *log = fopen("cabin_log.txt", "a"); // Mo file log de ghi
    if (log) { // Kiem tra file log co mo duoc khong
        fprintf(log, "%.1f°C, %ld Pa, %.1f m, duration: %.1f s\n", tempC, pressure, altitude, duration); // Ghi du lieu
        fclose(log); // Dong file log
    } else {
        perror("Cannot open log file"); // In thong bao loi
    }
}

// Ham mo phong xuat do cao cho he thong
void output_altitude_for_system(double altitude) { // Ham xuat do cao
    printf("[SYSTEM OUTPUT] Current altitude: %.1f m\n", altitude); // In do cao
}

//Author: Nguyen TRung Nhan
// Date: 2024-01-01
// Version: 1.0


int main() { // Ham chinh cua chuong trinh
    signal(SIGINT, signal_handler); // Dang ky ham xu ly tin hieu SIGINT

    // Khoi tao wiringPi
    wiringPiSetupPhys(); // Thiet lap WiringPi su dung so chan vat ly

    // Khoi tao soft PWM cho LED RGB
    softPwmCreate(RED_PIN,   0, 100); // Khoi tao PWM cho LED do
    softPwmCreate(GREEN_PIN, 0, 100); // Khoi tao PWM cho LED xanh la
    softPwmCreate(BLUE_PIN,  0, 100); // Khoi tao PWM cho LED xanh duong

    // Dat LED mac dinh la WHITE
    softPwmWrite(RED_PIN,   100); // Dat LED do sang toi da
    softPwmWrite(GREEN_PIN, 100); // Dat LED xanh la sang toi da
    softPwmWrite(BLUE_PIN,  100); // Dat LED xanh duong sang toi da

    int device = open("/dev/i2c-bmp180", O_RDWR); // Mo thiet bi I2C BMP180
    if (device < 0) { // Kiem tra loi
        perror("Cannot open device"); // In thong bao loi
        return errno; // Thoat chuong trinh voi ma loi
    }

    // Khoi tao SPI va MAX7219
    loadSPI(); // Khoi tao giao tiep SPI
    initMax7219(); // Khoi tao chip MAX7219

    long temp = 0, pres = 0; // Bien luu nhiet do va ap suat
    int oss = -1, res = 0; // Bien luu muc do lay mau (OSS) va ket qua IOCTL
    long previous_pres = 0; // Bien luu ap suat truoc do
    double previous_alti = 0; // Bien luu do cao truoc do
    double speed = 40.0; // Bien toc do ban dau
    time_t start_time, current_time; // Bien luu thoi gian bat dau va hien tai
    bool skip_leak = false; // Co bo qua canh bao ro ri

    // Mang luu lich su ap suat va do cao
    double pressure_history[MOVING_AVERAGE_WINDOW] = {0}; // Mang luu lich su ap suat
    double altitude_history[MOVING_AVERAGE_WINDOW] = {0}; // Mang luu lich su do cao
    int history_index = 0; // Chi so hien tai trong mang lich su
    int history_size = 0; // Kich thuoc lich su

    double sea_level_pressure = SEA_LEVEL_PRESSURE; // Ap suat tham chieu ban dau
    time(&start_time); // Bat dau do thoi gian

    ioctl(device, IOCTL_BMP180_COEF); // Lay he so cam bien BMP180
    ioctl(device, IOCTL_BMP180_DBGMSG); // Bat che do debug cho BMP180

    while (keep_running) { // Vong lap chinh chay khi keep_running la true
        printf("----- DOC DU LIEU BMP180 -----\n"); // In tieu de

        // Doc du lieu tu cam bien
        res = ioctl(device, IOCTL_BMP180_GET_TEMP, &temp); // Doc nhiet do
        if (res < 0) { // Kiem tra loi
            perror("Failed to get temperature"); // In thong bao loi
            close(device); // Dong thiet bi
            return errno; // Thoat chuong trinh voi ma loi
        }
        double tempC = temp * 0.1; // Chuyen doi nhiet do sang do C

        res = ioctl(device, IOCTL_BMP180_GET_PRES, &pres); // Doc ap suat
        if (res < 0) { // Kiem tra loi
            perror("Failed to get pressure"); // In thong bao loi
            close(device); // Dong thiet bi
            return errno; // Thoat chuong trinh voi ma loi
        }
        ioctl(device, IOCTL_BMP180_GET_OSS, &oss); // Lay muc do lay mau OSS

        // Mo phong dao dong ±10 Pa cho ap suat
        pres += (rand() % 20 - 10); // Them dao dong ngau nhien

        // Tinh do cao voi ap suat tham chieu dong
        // Tinh toan do cao(ap dung cho toan bo xe, bao gom dau xe va khung xe)
        double altitude = 44330.0 * (1.0 - pow(pres / sea_level_pressure, 0.1903)); // Tinh do cao

        // Cap nhat ap suat tham chieu dong
        sea_level_pressure = get_sea_level_pressure(); // Lay ap suat muc nuoc bien

        // Luu du lieu vao lich su
        pressure_history[history_index] = pres; // Luu ap suat vao mang
        altitude_history[history_index] = altitude; // Luu do cao vao mang
        history_index = (history_index + 1) % MOVING_AVERAGE_WINDOW; // Cap nhat chi so vong
        if (history_size < MOVING_AVERAGE_WINDOW) history_size++; // Tang kich thuoc lich su

        // Tinh trung binh truot
        double smoothed_pressure = moving_average(pressure_history, history_size, MOVING_AVERAGE_WINDOW); // Tinh ap suat trung binh
        double smoothed_altitude = moving_average(altitude_history, history_size, MOVING_AVERAGE_WINDOW); // Tinh do cao trung binh

        // Tinh thoi gian troi qua
        time(&current_time); // Lay thoi gian hien tai
        double duration = difftime(current_time, start_time); // Tinh thoi gian troi qua

        printf("Temp: %.1f 'C   Pres: %ld Pa (%.1f mmHg)   OSS=%d\n", tempC, pres, pres / 133.3, oss); // In nhiet do, ap suat, OSS
        printf("Altitude: %.1f m\n", altitude); // In do cao
        printf("Smoothed Pressure: %.1f Pa   Smoothed Altitude: %.1f m\n", smoothed_pressure, smoothed_altitude); // In ap suat va do cao trung binh
        printf("Sea Level Pressure: %.1f Pa\n", sea_level_pressure); // In ap suat muc nuoc bien

        // Hien thi nhiet do tren MAX7219
        displayTemp(tempC); // Hien thi nhiet do (vi du: 25.7°C)
        check_temperature_warning(device, tempC, vehicle_is_parked); // Kiem tra canh bao nhiet do

        // Kiem tra va dieu chinh
        if (previous_pres != 0) { // Neu co du lieu ap suat truoc do
            printf(">>> Ap suat da thay doi tu %ld Pa sang %ld Pa.\n", previous_pres, pres); // In thong bao thay doi ap suat
            long diff = (long)smoothed_pressure - previous_pres; // Tinh chenh lech ap suat
            printf(">>> Chenh lech ap suat: %ld Pa.\n", diff); // In chenh lech

            // Truong hop A & B: Phat hien xu huong ap suat
            detect_pressure_trend(pressure_history, history_size, TREND_WINDOW); // Kiem tra xu huong
            
            // Truong hop C: Phat hien dao dong bat thuong
            detect_pressure_fluctuation(pressure_history, history_size, MOVING_AVERAGE_WINDOW, smoothed_pressure); // Kiem tra dao dong
            
            // Truong hop E: Phat hien ap suat khong doi
            detect_no_pressure_change(pressure_history, history_size, MOVING_AVERAGE_WINDOW, duration, &skip_leak); // Kiem tra ap suat khong doi
            
            // Truong hop D: Phat hien ap suat giam nhanh
            compensate_air_pressure(device, (long)smoothed_pressure - previous_pres, &skip_leak); // Can bang ap suat
            
            // Canh bao ap suat, co the bo qua ro ri
            check_pressure_warning((long)smoothed_pressure, skip_leak); // Kiem tra canh bao ap suat
            check_altitude_warning(smoothed_altitude); // Kiem tra canh bao do cao
            output_altitude_for_system(smoothed_altitude); // Xuat do cao
            detect_pressure_change(previous_pres, (long)smoothed_pressure); // Kiem tra thay doi ap suat
            detect_tunnel_entry(previous_alti, smoothed_altitude, previous_pres, (long)smoothed_pressure); // Kiem tra vao/ra ham
            auto_adjust_air_system(device, diff, tempC); // Dieu chinh dieu hoa
            optimize_fuel_and_air_system(smoothed_altitude); // Toi uu phun nhien lieu
            detect_cabin_tampering(previous_pres, (long)smoothed_pressure, vehicle_is_parked); // Kiem tra xam nhap
            
        } else {
            printf(">>> Day la lan doc dau tien, khong co ap suat truoc do.\n"); // In thong bao lan doc dau tien
        }

        // Ghi log
        log_to_file(tempC, pres, altitude, duration); // Ghi du lieu vao file log
        adjust_air_suspension(device, smoothed_altitude); // Dieu chinh he thong treo

        // Kiem tra toc do va dieu chinh ap suat
        res = ioctl(device, IOCTL_BMP180_GET_SPEED, &speed); // Lay toc do
        if (res < 0) { // Kiem tra loi
            perror("Không thể lấy tốc độ"); // In thong bao loi
            speed = 0.0; // Dat toc do ve 0
        }
        control_pressure_at_high_speed(device, smoothed_pressure, speed); // Dieu chinh ap suat tai toc do cao

        // Cap nhat du lieu truoc
        previous_pres = (long)smoothed_pressure; // Cap nhat ap suat truoc do
        previous_alti = smoothed_altitude; // Cap nhat do cao truoc do

        // Doc lai ap suat voi OSS cao hon
        oss = 3; // Dat muc do lay mau cao
        ioctl(device, IOCTL_BMP180_SET_OSS, &oss); // Thiet lap OSS
        res = ioctl(device, IOCTL_BMP180_GET_PRES, &pres); // Doc ap suat
        if (res < 0) { // Kiem tra loi
            perror("Failed to get high-OSS pressure"); // In thong bao loi
            close(device); // Dong thiet bi
            return errno; // Thoat chuong trinh voi ma loi
        }
        ioctl(device, IOCTL_BMP180_GET_OSS, &oss); // Lay lai OSS
        printf("Pres (OSS=3): %ld Pa (%.1f mmHg)\n", pres, pres / 133.3); // In ap suat voi OSS cao

        altitude = 44330.0 * (1.0 - pow(pres / sea_level_pressure, 0.1903)); // Tinh do cao voi OSS cao
        printf("Altitude (OSS=3): %.1f m\n", altitude); // In do cao

        ioctl(device, IOCTL_BMP180_DBGMSG); // In thong bao debug

        sleep(5); // Cho 5 giay truoc khi doc tiep
        printf("\033[H\033[J"); // Xoa man hinh console
    }

    printf("Ket thuc chuong trinh.\n"); // In thong bao ket thuc
    close(device); // Dong thiet bi
    return 0; // Thoat chuong trinh
}

//Author: Nguyen TRung Nhan
// Date: 2024-01-01
// Version: 1.0


#include <iostream>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <chrono>
#include <thread>
#include <sched.h>

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

int main() {
    // 1. Elevate thread to Real-Time Priority
    struct sched_param param;
    param.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        std::cerr << "Warning: Failed to set SCHED_FIFO. Run with sudo." << std::endl;
    }

    // 2. Open I2C Bus 7 on Jetson Nano
    int file = open("/dev/i2c-7", O_RDWR);
    if (file < 0 || ioctl(file, I2C_SLAVE, MPU6050_ADDR) < 0) {
        std::cerr << "Failed to access MPU6050 on /dev/i2c-7" << std::endl;
        return 1;
    }

    // 3. Wake up MPU6050 (Use PLL with X-axis gyroscope reference for stability)
    uint8_t wake_buf[2] = {PWR_MGMT_1, 0x01};
    write(file, wake_buf, 2);
    usleep(10000);

    // 4. Enable Hardware Digital Low-Pass Filter (DLPF_CFG = 4 -> 21Hz cutoff)
    // This physically filters out high-frequency chassis vibration from the spinning RPLIDAR!
    uint8_t dlpf_buf[2] = {CONFIG, 0x04};
    write(file, dlpf_buf, 2);

    // 5. Explicitly lock Gyro to +/- 250 deg/s (131.0 LSB/deg/s)
    uint8_t gyro_buf[2] = {GYRO_CONFIG, 0x00};
    write(file, gyro_buf, 2);

    // 6. Explicitly lock Accel to +/- 2g (16384.0 LSB/g)
    uint8_t accel_buf[2] = {ACCEL_CONFIG, 0x00};
    write(file, accel_buf, 2);

    // Disable stdio buffering for instant pipe throughput
    std::ios_base::sync_with_stdio(false);
    std::cout << "Time_ns,ax,ay,az,gx,gy,gz\n" << std::flush;

    uint8_t reg = ACCEL_XOUT_H;
    uint8_t data[14];

    while (true) {
        if (write(file, &reg, 1) != 1 || read(file, data, 14) != 14) {
            continue;
        }

        auto ts = std::chrono::steady_clock::now().time_since_epoch().count();

        // Safe unsigned-to-signed 16-bit reconstruction on ARM64
        int16_t ax = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
        int16_t ay = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);
        int16_t az = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]);
        int16_t gx = static_cast<int16_t>((static_cast<uint16_t>(data[8]) << 8) | data[9]);
        int16_t gy = static_cast<int16_t>((static_cast<uint16_t>(data[10]) << 8) | data[11]);
        int16_t gz = static_cast<int16_t>((static_cast<uint16_t>(data[12]) << 8) | data[13]);

        std::cout << ts << "," << ax << "," << ay << "," << az
                  << "," << gx << "," << gy << "," << gz << "\n";

        // 200 Hz loop (5000 us)
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    close(file);
    return 0;
}
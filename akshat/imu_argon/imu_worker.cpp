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

// Helper function that checks the return value of write() to satisfy GCC and catch loose I2C wires
bool write_reg(int file, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return (write(file, buf, 2) == 2);
}

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

    // 3. Configure MPU6050 registers with strict return-value verification
    if (!write_reg(file, PWR_MGMT_1, 0x01)) { // Wake up + PLL with X-axis gyro reference
        std::cerr << "Failed to wake MPU6050" << std::endl;
        return 1;
    }
    usleep(10000);

    if (!write_reg(file, CONFIG, 0x04) ||       // Hardware DLPF = 21 Hz (filters RPLIDAR vibration)
        !write_reg(file, GYRO_CONFIG, 0x00) ||  // +/- 250 deg/s
        !write_reg(file, ACCEL_CONFIG, 0x00)) { // +/- 2g
        std::cerr << "Failed to configure MPU6050 DLPF/scales" << std::endl;
        return 1;
    }

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

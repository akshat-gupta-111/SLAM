#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <chrono>
#include <thread>
#include <sched.h>

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B

int main() {
    // 1. Elevate this thread to Real-Time Industrial Priority
    struct sched_param param;
    param.sched_priority = 80; // High priority (1-99)
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        std::cerr << "Failed to set Real-Time priority. Run with sudo!" << std::endl;
    }

    // 2. Open the I2C Bus
    int file = open("/dev/i2c-7", O_RDWR);
    if (file < 0 || ioctl(file, I2C_SLAVE, MPU6050_ADDR) < 0) {
        std::cerr << "Failed to access MPU6050 on i2c-7" << std::endl;
        return 1;
    }

    // 3. Wake up sensor
    char wake_buf[2] = {PWR_MGMT_1, 0x00};
    write(file, wake_buf, 2);
    
    std::cout << "Time_ns,ax,ay,az,gx,gy,gz" << std::endl;

    char reg = ACCEL_XOUT_H;
    char data[14];

    while (true) {
        // Continuous block read (Fastest way over I2C)
        write(file, &reg, 1);
        read(file, data, 14);

        // Hardware-accurate timestamp recorded immediately after I2C transfer
        auto ts = std::chrono::steady_clock::now().time_since_epoch().count();

        // Parse signed 16-bit integers
        int16_t ax = (data[0] << 8) | data[1];
        int16_t ay = (data[2] << 8) | data[3];
        int16_t az = (data[4] << 8) | data[5];
        int16_t gx = (data[8] << 8) | data[9];
        int16_t gy = (data[10] << 8) | data[11];
        int16_t gz = (data[12] << 8) | data[13];

        // Stream clean CSV out to a pipeline or shared memory
        std::cout << ts << "," << ax << "," << ay << "," << az 
                  << "," << gx << "," << gy << "," << gz << "\n";

        // Deterministic sleep (e.g., 200 Hz = 5000 microseconds)
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
    close(file);
    return 0;
}

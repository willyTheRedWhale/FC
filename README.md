Features:
1. Read IMU (LSM6DSOXS) output using IRQ at 6667 HZ over SPI
2. Read Magnetometer (MMC56x3) output at 255Hz
3. Raed GPS data
4. Split tasks into two cores
5. Share data between the two cores of rp2350
6. Core 0 takes are of serial, slower tasks like magnetometer, gps, output of data
7. Core 1 takes care of imu, sensor fusion, and writes to nrf24l01
8. Output stable roll, pitch
9. Output stable yaw when parallel to ground.
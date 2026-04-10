// tool.h
#ifndef IMUTOOL_H
#define IMUTOOL_H
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>
#include <functional>
#include <atomic>
#include <mutex>

class SerialPort
{
private:
    int fd;

public:
    SerialPort() : fd(-1) {}

    bool open(const std::string &port, int baudrate = 9600)
    {
        fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0)
        {
            return false;
        }

        termios tty;
        if (tcgetattr(fd, &tty) != 0)
        {
            ::close(fd);
            fd = -1;
            return false;
        }

        // 设置波特率
        speed_t speed;
        switch (baudrate)
        {
        case 9600:
            speed = B9600;
            break;
        case 19200:
            speed = B19200;
            break;
        case 38400:
            speed = B38400;
            break;
        case 57600:
            speed = B57600;
            break;
        case 115200:
            speed = B115200;
            break;
        default:
            speed = B9600;
            break;
        }
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        // 设置数据位、停止位、校验位
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8数据位
        tty.c_cflag &= ~PARENB;                     // 无校验
        tty.c_cflag &= ~CSTOPB;                     // 1停止位
        tty.c_cflag &= ~CRTSCTS;                    // 无硬件流控

        // 设置原始模式
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_oflag &= ~OPOST;

        // 设置超时
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5; // 0.5秒超时

        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
            ::close(fd);
            fd = -1;
            return false;
        }

        return true;
    }

    bool write(const std::vector<uint8_t> &data)
    {
        ssize_t result = ::write(fd, data.data(), data.size());
        return result == static_cast<ssize_t>(data.size());
    }

    std::vector<uint8_t> read()
    {
        std::vector<uint8_t> buffer(256);
        ssize_t bytesRead = ::read(fd, buffer.data(), buffer.size());
        if (bytesRead > 0)
        {
            buffer.resize(bytesRead);
        }
        else
        {
            buffer.clear();
        }
        return buffer;
    }

    int available()
    {
        int bytes = 0;
        if (ioctl(fd, FIONREAD, &bytes) == 0)
        {
            return bytes;
        }
        return 0;
    }

    void close()
    {
        if (fd >= 0)
        {
            ::close(fd);
            fd = -1;
        }
    }

    ~SerialPort()
    {
        close();
    }
};

class SensorParser
{
public:
    static bool parseSensorData(const std::string &dataHex, double &x, double &y)
    {
        if (dataHex.length() < 26)
        {
            // std::cout << "长度不够" << std::endl;
            return false;
        }

        try
        {
            // 提取X轴和Y轴的4字节数据
            std::string x_data = dataHex.substr(6, 8);  // X轴的4字节十六进制
            std::string y_data = dataHex.substr(14, 8); // Y轴的4字节十六进制

            // 解析X轴数据
            uint32_t x_value = parseAxisData(x_data);
            // 解析Y轴数据
            uint32_t y_value = parseAxisData(y_data);

            // 转换为有符号整数
            int32_t x_signed = toSigned32(x_value);
            int32_t y_signed = toSigned32(y_value);

            // 应用校准和缩放
            x = (x_signed - 9000) * 0.01;
            y = (y_signed - 9000) * 0.01;

            return true;
        }
        catch (const std::exception &e)
        {
            return false;
        }
    }

private:
    static uint32_t parseAxisData(const std::string &hexData)
    {
        if (hexData.length() != 8)
        {
            throw std::runtime_error("十六进制数据长度错误");
        }

        // 将十六进制字符串转换为字节
        std::vector<uint8_t> bytes;
        for (size_t i = 0; i < 8; i += 2)
        {
            std::string byteString = hexData.substr(i, 2);
            uint8_t byte = static_cast<uint8_t>(std::stoul(byteString, nullptr, 16));
            bytes.push_back(byte);
        }

        // 前2字节是低16位(小端序)，后2字节是高16位
        uint16_t low = bytes[0] | (bytes[1] << 8);
        uint16_t high = bytes[2] | (bytes[3] << 8);

        return (static_cast<uint32_t>(high) << 16) | low;
    }

    static int32_t toSigned32(uint32_t value)
    {
        if (value & 0x80000000)
        {
            return static_cast<int32_t>(value - 0x100000000);
        }
        return static_cast<int32_t>(value);
    }
};

std::string bytesToHex(const std::vector<uint8_t> &data)
{
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (uint8_t byte : data)
    {
        ss << std::setw(2) << static_cast<int>(byte);
    }
    return ss.str();
}

// 传感器数据获取类
class SensorReader
{
private:
    SerialPort ser;
    std::vector<uint8_t> cmd = {0x04, 0x03, 0x00, 0x02, 0x00, 0x04, 0xE5, 0x9C};
    bool initialized = false;
    std::atomic<int> idx_{0};
    std::mutex mutex_;
    std::vector<std::pair<double, double>> data_buffer;
    std::atomic<bool> running{false};
    std::thread data_thread;


public:
    /**
     * @brief 初始化传感器
     * @param port 串口设备路径，如 "/dev/ttyUSB0"
     * @return true-成功, false-失败
     */
    bool init(const double& tt, const std::string &port = "/dev/ttyUSB0")
    {
        if (ser.open(port, 9600))
        {
            initialized = true;
            data_buffer.resize(4, {0.0, 0.0});
            startPeriodicReading(tt);
            return true;
        }
        return false;
    }

    /**
     * @brief 获取传感器数据
     * @param x 输出的X轴数据
     * @param y 输出的Y轴数据
     * @param timeout_ms 超时时间(毫秒)
     * @return true-成功获取数据, false-获取失败
     */
    bool getSensorData(double &x, double &y, int timeout_ms = 1000)
    {
        if (!initialized)
        {
            std::cout << "传感器未初始化" << std::endl;
            return false;
        }

        // 发送指令
        if (!ser.write(cmd))
        {
            std::cout << "发送指令失败" << std::endl;
            return false;
        }

        // 等待响应
        int waited = 0;
        while (ser.available() <= 0 && waited < timeout_ms)
        {
            usleep(10000); // 10ms
            waited += 10;
        }

        if (ser.available() <= 0)
        {
            std::cout << "等待传感器响应超时" << std::endl;
            return false;
        }

        // 读取数据
        std::vector<uint8_t> data = ser.read();
        if (data.empty())
        {
            std::cout << "读取数据为空" << std::endl;
            return false;
        }

        std::string data_hex = bytesToHex(data);

        // 解析数据
        if (SensorParser::parseSensorData(data_hex, x, y))
        {
            std::lock_guard<std::mutex> lock(mutex_);
            data_buffer[idx_] = {x, y};
            idx_ = (idx_ + 1) % 4;
            return true;
        }
        else
        {
            // std::cout << "解析数据失败: " << data_hex << std::endl;
            return false;
        }
    }

    /**
     * @brief 开始周期性数据采集
     * @param period_ms 采集周期(毫秒)
     * @param callback 数据回调函数
     * @return true-成功启动, false-启动失败
     */
    bool startPeriodicReading(const double& t)
    {
        if (!initialized)
        {
            std::cout << "传感器未初始化" << std::endl;
            return false;
        }

        if (running)
        {
            std::cout << "数据采集已在运行" << std::endl;
            return false;
        }

        running = true;
        data_thread = std::thread([this,t]()
                                  { this->dataReadingThread(t/4); });

        return true;
    }

    /**
     * @brief 停止周期性数据采集
     */
    void stopPeriodicReading()
    {
        running = false;
        if (data_thread.joinable())
        {
            data_thread.join();
        }
    }
    void getDataBufferIdx(double& x,double& y){
        std::lock_guard<std::mutex> lock(mutex_);
        x = data_buffer[idx_].first;
        y = data_buffer[idx_].second;
    }
    /**
     * @brief 获取缓冲区中的数据的平均值
     */
    void getDataBufferAvg(double &x_avg, double &y_avg)
    {
        int n = 0;
        for (const auto &[x, y] : data_buffer)
        {
            // 说明是初始化的
            if(abs(x)<0.0001&&abs(y)<0.0001)
                continue;
            x_avg += x;
            y_avg += y;
            n++;
        }
        if(n==0){
            x_avg = 0.0;
            y_avg = 0.0;
            return;
        }
        x_avg /= n;
        y_avg /= n;
        // std::cout << data_buffer[idx_].first << '\t' << data_buffer[idx_].second << std::endl;
        // std::cout<<x_avg << '\t' <<y_avg << std::endl;
    }

    /**
     * @brief 清空数据缓冲区
     */
    void clearBuffer()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data_buffer.clear();
    }

    /**
     * @brief 关闭传感器
     */
    void close()
    {
        stopPeriodicReading();
        ser.close();
        initialized = false;
    }

    ~SensorReader()
    {
        close();
    }

private:
    /**
     * @brief 数据采集线程函数
     * @param period_ms 采集周期(毫秒)
     */
    void dataReadingThread(int period_ms)
    {
        auto last_time = std::chrono::steady_clock::now();

        while (running)
        {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time);

            if (elapsed.count() >= period_ms)
            {
                double x, y;
                if (getSensorData(x, y))
                {
                    uint64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                                             std::chrono::system_clock::now().time_since_epoch())
                                             .count();
                    // std::cout << x <<'\t' << y <<std::endl;
                }
                last_time = current_time;
            }

            // 短暂休眠，避免过度占用CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
};

#endif
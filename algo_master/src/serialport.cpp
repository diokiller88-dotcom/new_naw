#include "serialport.hpp"
#include <chrono>
#include <thread>
#include <cstring>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>

namespace serial {

bool NautilusSerialPort::m_Open(const char* portname, int baudrate, int parity, int databit, int stopbit)
{
    pHandle = -1;
    pHandle = ::open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (pHandle == -1) return false;

    struct termios options;
    if (tcgetattr(pHandle, &options) < 0) return false;

    switch (baudrate) {
    case 4800:   cfsetispeed(&options, B4800); cfsetospeed(&options, B4800); break;
    case 9600:   cfsetispeed(&options, B9600); cfsetospeed(&options, B9600); break;
    case 19200:  cfsetispeed(&options, B19200); cfsetospeed(&options, B19200); break;
    case 38400:  cfsetispeed(&options, B38400); cfsetospeed(&options, B38400); break;
    case 57600:  cfsetispeed(&options, B57600); cfsetospeed(&options, B57600); break;
    case 115200: cfsetispeed(&options, B115200); cfsetospeed(&options, B115200); break;
    default: return false;
    }

    switch (parity) {
    case 0: options.c_cflag &= ~PARENB; options.c_cflag &= ~INPCK; break;
    case 1: options.c_cflag |= PARENB; options.c_cflag |= PARODD; options.c_cflag |= INPCK; options.c_cflag |= ISTRIP; break;
    case 2: options.c_cflag |= PARENB; options.c_cflag &= ~PARODD; options.c_cflag |= INPCK; options.c_cflag |= ISTRIP; break;
    default: return false;
    }

    switch (databit) {
    case 5: options.c_cflag &= ~CSIZE; options.c_cflag |= CS5; break;
    case 6: options.c_cflag &= ~CSIZE; options.c_cflag |= CS6; break;
    case 7: options.c_cflag &= ~CSIZE; options.c_cflag |= CS7; break;
    case 8: options.c_cflag &= ~CSIZE; options.c_cflag |= CS8; break;
    default: return false;
    }

    switch (stopbit) {
    case 1: options.c_cflag &= ~CSTOPB; break;
    case 2: options.c_cflag |= CSTOPB; break;
    default: return false;
    }

    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_iflag &= ~(ICRNL | IGNCR);

    if ((tcsetattr(pHandle, TCSANOW, &options)) != 0) return false;
    return true;
}

void NautilusSerialPort::m_Close()
{
    if (pHandle != -1) {
        ::close(pHandle);
        pHandle = -1;
    }
}

int NautilusSerialPort::m_Send(const void* buf, int len)
{
    int sendCount = 0;
    if (pHandle != -1) {
        const char* buffer = (char*)buf;
        size_t length = len;
        ssize_t tmp;
        while (length > 0) {
            if ((tmp = write(pHandle, buffer, length)) <= 0) {
                if (tmp < 0 && errno == EINTR) tmp = 0;
                else break;
            }
            length -= tmp;
            buffer += tmp;
        }
        sendCount = len - length;
    }
    return sendCount;
}

int NautilusSerialPort::m_Receive(void* buf, int maxlen)
{
    int receiveCount = ::read(pHandle, buf, maxlen);
    if (receiveCount < 0) receiveCount = 0;
    return receiveCount;
}

bool NautilusSerialPort::OpenPort(const std::string& portname, int baudrate, int parity, int databit, int stopbit, int synchronizeflag)
{
    m_Portname = portname;
    m_Baudrate = baudrate;
    m_Parity = parity;
    m_Databit = databit;
    m_Stopbit = stopbit;
    m_Synchronizeflag = synchronizeflag;

    if (m_Open(portname.c_str(), baudrate, parity, databit, stopbit)) {
        port_available = true;
        last_received = std::chrono::steady_clock::now();
        return true;
    }
    port_available = false;
    return false;
}

void NautilusSerialPort::ReadRawBuf()
{
    RawBufRecv rawBuf;
    while (rclcpp::ok()) {
        if (!port_available) {
            std::this_thread::sleep_for(std::chrono::milliseconds(serial_monitor_interval_ms));
            continue;
        }

        rawBuf.fill(0);
        if (m_Receive(rawBuf.data(), rawBuf.size()) > 0) {
            msgRawBufs.Push(rawBuf);
            last_received = std::chrono::steady_clock::now();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(serial_read_interval_ms));
        }
    }
}

void NautilusSerialPort::ProcRawBuf()
{
    NavRecvFromPLC payload;
    RawBufRecv rawBuf;
    while (rclcpp::ok()) {
        if (msgRawBufs.Pop(rawBuf)) {
            for (size_t i = 0; i < rawBuf.size();) {
                size_t j = i + recv_msg_size;
                if (j > rawBuf.size()) break;

                if (rawBuf[i] == 0xA5 && rawBuf[j - 1] == 0xAA) {
                    std::memcpy(&payload, rawBuf.data() + i, recv_msg_size);
                    msgSerialRecv.Push(payload);
                    i = j;
                } else {
                    ++i;
                }
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(serial_proc_interval_ms));
        }
    }
}

bool NautilusSerialPort::Send(const Nav2PLCSend& payload)
{
    if (!port_available) return false;
    uint8_t sendFrame[send_buf_size];
    memcpy(sendFrame, &payload, send_buf_size);
    return m_Send(sendFrame, sizeof(sendFrame)) == send_buf_size;
}

void NautilusSerialPort::CheckAndReconnect()
{
    while (rclcpp::ok()) {
        bool needs_reconnect = false;

        if (!port_available) {
            needs_reconnect = true;
        } else {
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_received);
            
            if (duration.count() >= serial_timeout_ms) {
                spdlog::warn("Serial connection timeout. Reconnecting to {}...", m_Portname);
                needs_reconnect = true;
            }
        }

        if (needs_reconnect) {
            port_available = false;
            m_Close();

            if (OpenPort(m_Portname, m_Baudrate, m_Parity, m_Databit, m_Stopbit, m_Synchronizeflag)) {
                spdlog::info("Successfully reconnected to {}", m_Portname);
            } else {
                std::this_thread::sleep_for(std::chrono::seconds(serial_reconnect_sleep_s));
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(serial_monitor_interval_ms));
        }
    }
}

void NautilusSerialPort::ClosePort() { m_Close(); }

} // namespace serial
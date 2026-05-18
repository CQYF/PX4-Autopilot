/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "Multipressure.hpp"

#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>

MULTIPRESSURE::MULTIPRESSURE(const char *port) :
    ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
    _sensor_multipressure_pub{ORB_ID(sensor_multipressure)}
{
    strncpy(_port, port, sizeof(_port) - 1);
    _port[sizeof(_port) - 1] = '\0';
}

MULTIPRESSURE::~MULTIPRESSURE()
{
    stop();

    perf_free(_sample_perf);
    perf_free(_comms_errors);
}

int
MULTIPRESSURE::init()
{
    int ret = 0;

    do {
        _fd = ::open(_port, O_RDWR | O_NOCTTY);

        if (_fd < 0) {
            PX4_ERR("Error opening fd");
            return -1;
        }

        unsigned speed = B115200;
        termios uart_config{};
        int termios_state{};

        tcgetattr(_fd, &uart_config);

        uart_config.c_oflag &= ~ONLCR;

        if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
            PX4_ERR("CFG: %d ISPD", termios_state);
            ret = -1;
            break;
        }

        if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
            PX4_ERR("CFG: %d OSPD", termios_state);
            ret = -1;
            break;
        }

        if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
            PX4_ERR("baud %d ATTR", termios_state);
            ret = -1;
            break;
        }

        uart_config.c_cflag |= (CLOCAL | CREAD);
        uart_config.c_cflag &= ~CSIZE;
        uart_config.c_cflag |= CS8;
        uart_config.c_cflag &= ~PARENB;
        uart_config.c_cflag &= ~CSTOPB;
        uart_config.c_cflag &= ~CRTSCTS;

        uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        uart_config.c_oflag &= ~OPOST;

        uart_config.c_cc[VMIN] = 1;
        uart_config.c_cc[VTIME] = 1;

    } while (0);

    ::close(_fd);
    _fd = -1;

    if (ret == PX4_OK) {
        start();
    }

    return ret;
}

int
MULTIPRESSURE::collect()
{
    perf_begin(_sample_perf);

    int64_t read_elapsed = hrt_elapsed_time(&_last_read);

    char readbuf[sizeof(_linebuf)] {};
    unsigned readlen = sizeof(readbuf) - 1;

    int ret = 0;
    float pressures[16] {};

    for (int i = 0; i < 16; i++) {
        pressures[i] = 0.0f;
    }

    int bytes_available = 0;
    ::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

    if (!bytes_available) {
        perf_end(_sample_perf);
        return 0;
    }

    const hrt_abstime timestamp_sample = hrt_absolute_time();

    do {
        ret = ::read(_fd, &readbuf[0], readlen);

        if (ret < 0) {
            PX4_ERR("read err: %d", ret);
            perf_count(_comms_errors);
            perf_end(_sample_perf);

            if (read_elapsed > (kCONVERSIONINTERVAL * 2)) {
                tcflush(_fd, TCIFLUSH);
                return ret;

            } else {
                return -EAGAIN;
            }
        }

        _last_read = hrt_absolute_time();

        for (int i = 0; i < ret; i++) {
            multipressure_parse(readbuf[i], _linebuf, &_linebuf_index, &_parse_state, pressures);
        }

        bytes_available -= ret;

    } while (bytes_available > 0);

    perf_end(_sample_perf);

    sensor_multipressure_s report{};
    report.timestamp = timestamp_sample;

    for (int i = 0; i < 16; i++) {
        report.pressure[i] = pressures[i];
    }

    _sensor_multipressure_pub.publish(report);

    return PX4_OK;
}

void
MULTIPRESSURE::start()
{
    ScheduleOnInterval(10_ms);
}

void
MULTIPRESSURE::stop()
{
    ScheduleClear();
}

void
MULTIPRESSURE::Run()
{
    if (_fd < 0) {
        _fd = ::open(_port, O_RDWR | O_NOCTTY);
    }

    if (collect() == -EAGAIN) {
        ScheduleClear();
        ScheduleOnInterval(10_ms, 66 * 10);
        return;
    }
}

void
MULTIPRESSURE::print_info()
{
    printf("Using port '%s'\n", _port);
    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);
}
#ifndef __SERIALLINK_H__
#define __SERIALLINK_H__

#pragma once

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdbool.h>
#include <string.h>

// --------------------------------------------------------------------------------------------------------------------
// Define the orders that can be sent and received
enum Order
{
    ORDER_HELLO = 0,
    ORDER_ALREADY_CONNECTED = 1,
    ORDER_SCAN = 2,
    ORDER_READ = 3,
    ORDER_START = 4,
    ORDER_STOP = 5,
    ORDER_ERROR = 255,
};
typedef enum Order order_t;
// --------------------------------------------------------------------------------------------------------------------
struct SerialLink
{
    speed_t m_baudrate;
    int fd;
    bool is_connected;
    char port[64];
};
typedef struct SerialLink link_t;
struct Sample
{
    uint64_t rom;
    float temperature;
    char timestamp[12];
};
typedef struct Sample sample_t;
// --------------------------------------------------------------------------------------------------------------------
int SerialLink_configure(link_t *pLink)
{
    struct termios tty;
    if (tcgetattr(pLink->fd, &tty) != 0)
    {
        perror("Error getting attributes");
        return -1;
    }

    cfsetospeed(&tty, pLink->m_baudrate); // Set output baud rate
    cfsetispeed(&tty, pLink->m_baudrate); // Set input baud rate

    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    tty.c_lflag &= ~ICANON; // Set raw mode
    tty.c_lflag &= ~(ECHO | ECHOE | ISIG);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable flow control
    tty.c_iflag &= ~(ICRNL | INLCR | IGNCR);

    tty.c_oflag &= ~OPOST; // Disable output processing

    tcflush(pLink->fd, TCIFLUSH);

    if (tcsetattr(pLink->fd, TCSANOW, &tty) != 0)
    {
        perror("Error setting attributes");
        return -1;
    }
    return 0;
}
// --------------------------------------------------------------------------------------------------------------------
void SerialLink_open_link(link_t *pLink, const char *_port, const speed_t _baudrate)
{
    pLink->fd = open(_port, O_RDWR | O_NOCTTY);
    if (pLink->fd == -1)
    {
        exit(1);
    }
    if (SerialLink_configure(pLink) != 0)
    {
        close(pLink->fd);
        exit(1);
    }
}
// --------------------------------------------------------------------------------------------------------------------
void SerialLink_initialize(link_t *pLink, const char *_port, const speed_t _baudrate)
{
    pLink->is_connected = false;
    strcpy(pLink->port, _port);
    SerialLink_open_link(pLink, pLink->port, pLink->m_baudrate);
}
// --------------------------------------------------------------------------------------------------------------------
void SerialLink_uninitialize(link_t *pLink)
{
    pLink->is_connected = false;
    if (pLink->fd != -1)
    {
        close(pLink->fd);
    }
}
// --------------------------------------------------------------------------------------------------------------------
bool SerialLink_is_data_available(link_t *pLink)
{
    int bytesAvailable;
    if (ioctl(pLink->fd, FIONREAD, &bytesAvailable) == -1)
    {
        SerialLink_open_link(pLink, pLink->port, pLink->m_baudrate);
        return false;
    }
    else
        return bytesAvailable > 0;
}
// --------------------------------------------------------------------------------------------------------------------
void SerialLink_write_object(link_t *pLink, const void *_object, const size_t _size)
{
    if (write(pLink->fd, _object, _size) == -1)
        SerialLink_open_link(pLink, pLink->port, pLink->m_baudrate);
}
// --------------------------------------------------------------------------------------------------------------------
void SerialLink_write_uint8(link_t *pLink, const uint8_t _value)
{
    SerialLink_write_object(pLink, &_value, sizeof(uint8_t));
}
// --------------------------------------------------------------------------------------------------------------------
void SerialLink_write_order(link_t *pLink, const order_t _order)
{
    SerialLink_write_uint8(pLink, (uint8_t)_order);
}
// --------------------------------------------------------------------------------------------------------------------
void SerialLink_read_object(link_t *pLink, void *_object, const size_t _size)
{
    if (read(pLink->fd, _object, _size) == -1)
        SerialLink_open_link(pLink, pLink->port, pLink->m_baudrate);
}
// --------------------------------------------------------------------------------------------------------------------
uint8_t SerialLink_read_uint8(link_t *pLink)
{
    uint8_t value;
    SerialLink_read_object(pLink, &value, sizeof(uint8_t));
    return value;
}
// --------------------------------------------------------------------------------------------------------------------
order_t SerialLink_read_order(link_t *pLink)
{
    return (order_t)SerialLink_read_uint8(pLink);
}
// --------------------------------------------------------------------------------------------------------------------
sample_t SerialLink_read_sample(link_t *pLink)
{
    sample_t sample;
    SerialLink_read_object(pLink, &sample, sizeof(sample_t));
    return sample;
}

// --------------------------------------------------------------------------------------------------------------------
void SerialLink_read_and_respond(link_t *pLink)
{
    if (SerialLink_is_data_available(pLink))
    {
        order_t order = SerialLink_read_order(pLink);
        if (order == ORDER_HELLO)
        {
            if (!pLink->is_connected)
            {
                pLink->is_connected = true;
                SerialLink_write_order(pLink, ORDER_HELLO);
            }
            else
            {
                SerialLink_write_order(pLink, ORDER_ALREADY_CONNECTED);
            }
        }
        else if (order == ORDER_ALREADY_CONNECTED)
        {
            pLink->is_connected = true;
        }
    }
}
// --------------------------------------------------------------------------------------------------------------------
void SerialLink_wait_for_connection(link_t *pLink)
{
    order_t order;
    SerialLink_write_order(pLink, ORDER_HELLO);
    while (!pLink->is_connected)
    {
        printf("Waiting for connection...\n");
        sleep(1);
        if (SerialLink_is_data_available(pLink))
        {
            order = SerialLink_read_order(pLink);
            if ((order == ORDER_HELLO) || (order == ORDER_ALREADY_CONNECTED))
            {
                pLink->is_connected = true;
            }
        }
        else
        {
            continue;
        }
    }
    printf("Connected...\n");
}
// --------------------------------------------------------------------------------------------------------------------

#endif //__SERIALLINK_H__

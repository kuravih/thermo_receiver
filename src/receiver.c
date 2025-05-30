#include "serial_link.h"

#include <stdatomic.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>

atomic_bool busy = true;

void sigint_handler(int signal)
{
    if (signal == SIGINT)
    {
        busy = false;
        printf("sigint_handler() - Terminating readout ...\n");
    }
}

int main(int argc, char const *argv[])
{
    signal(SIGINT, sigint_handler);

    link_t link;

    SerialLink_initialize(&link, "/dev/ttyACM0", B115200);

    SerialLink_wait_for_connection(&link);

    SerialLink_write_order(&link, ORDER_STOP);

    SerialLink_write_order(&link, ORDER_START);

    tcflush(link.fd, TCIFLUSH);

    sample_t sample;
    while (busy && link.is_connected)
    {
        sample = SerialLink_read_sample(&link);
        printf("rom:%#018lx, temperature:%+9.4f, timestamp:%s\n", sample.rom, sample.temperature, sample.timestamp);
    }

    // SerialLink_write_order(&link, ORDER_STOP);

    // SerialLink_uninitialize(&link);

}

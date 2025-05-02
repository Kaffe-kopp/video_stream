// C library headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image/stb_image.h"

// +9Cs~f?Rfn/Jxn~L*j,<"p!

int main() {
    char com_port[] = "/dev/ttyUSB0";
    //char com_port[] = "COM3";

    int serial_port = open(com_port, O_RDWR);

    // Check for errors
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    struct termios tty;

    if (tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    //tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    //tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)
    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);


    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }


    int width, height, channels;

    char img_path[] = "cframes/1000.jpg";

    stbi_info(img_path, &width, &height, &channels);


    int img_size = width*height;
    int img_size_bin = (int)(img_size/8);
    int send_buffer = width;

    unsigned char *img_gray = malloc(img_size);
    unsigned char *img_bin = malloc(img_size_bin);

    float time_between_frames = 0.75/10;
    clock_t prev_time = clock();

    for (int frame = 50; frame < 6510; frame += 3) {

        sprintf(img_path, "cframes/%d.jpg", frame);

        unsigned char *img_rgb = stbi_load(img_path, &width, &height, &channels, 3);


        if (img_rgb == NULL) {
            printf("Error in loading the image\n");exit(1);
        }

        for (unsigned char *p = img_rgb, *pg = img_gray; p != img_rgb + img_size*channels; p += channels, pg++) {
        *pg = (uint8_t)((128 < (uint8_t)((*p + *(p + 1) + *(p + 2))/3.0)) ? 1:0);
        }

        int j = 0;
        uint8_t pixel = 0;
        for (int i = 0; i < img_size; i++) {
             pixel = pixel|img_gray[i];
              if ((i+1)%8 == 0) {
                img_bin[j] = pixel;
                pixel = 0;
                j++;
            }
             pixel = pixel << 1;
        }

        while (((float)clock() - prev_time) < time_between_frames);
        prev_time = clock();

        for (int k = 0; k < img_size_bin; k++) {
            uint8_t to_send = img_bin[k];
            int sent_bytes = write(serial_port, &to_send, sizeof(to_send));
           usleep(50);
        }
    }
    free(img_gray);
    free(img_bin);
    return 0;
}

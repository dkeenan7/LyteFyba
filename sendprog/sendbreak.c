#include <termios.h>
#include <fcntl.h>
#include <stdio.h>

int main(int argc,char** argv) {
  if (argc<2) {
    printf("Usage: One parameter, the device to send the "
           "break to. Example: %s /dev/tty.usbmodem622 \n",argv[0]);
  } else {
    int tty_fd = open(argv[1], O_RDWR | O_NONBLOCK);
    tcsendbreak(tty_fd, 0);
  }
}

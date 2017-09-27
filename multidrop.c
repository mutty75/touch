#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <asm/ioctls.h>

#ifndef PARMD
#define PARMD   0100000
#endif
#ifndef SENDA
#define SENDA   0200000
#endif

int readn(int fd, void *vptr, size_t n)
{
        int nleft = n, nread;
        char *ptr = vptr;

        while (nleft > 0) {
                nread = read(fd, ptr, nleft);
                if (nread < 0) {
                        if (errno != EAGAIN)
                                return -1;
                        nread = 0;
                } else if (nread == 0)
                        break;

                nleft -= nread;
                ptr += nread;
        }

        return n - nleft;
}

int writen(int fd, void *vptr, size_t n)
{
        int nleft = n, nwritten;
        char *ptr = vptr;

        while (nleft > 0) {
                nwritten = write(fd, ptr, nleft);
                if (nwritten < 0) {
                        if (errno != EAGAIN)
                                return -1;
                        nwritten = 0;
                } else if (nwritten == 0)
                        break;

                nleft -= nwritten;
                ptr += nwritten;
        }

        return n - nleft;
}

int main(int argc, char *argv[])
{
        char *dev = "/dev/ttyS1";
	int fd;
	struct serial_rs485 rs485conf;
	struct termios term;
	char buf[128];
	int i, n;
	int ret;

        if (argc > 1)
                dev = argv[1];

        /* Open the serial device */
        ret = open(dev, O_RDWR|O_NOCTTY);
	if (ret < 0) {
		perror("unable to open (%m)");
		exit(-1);
	}
	fd = ret;

	/* Force rs485 mode */
        memset(&rs485conf, 0, sizeof(rs485conf));
        ret = ioctl(fd, TIOCGRS485, &rs485conf);
	if (ret < 0) {
		perror("unable to get rs485 conf (%m)");
                exit(-1);
        }

	rs485conf.flags |= SER_RS485_ENABLED;
	rs485conf.flags &= ~SER_RS485_RX_DURING_TX;
	rs485conf.delay_rts_after_send = 0;

	ret = ioctl(fd, TIOCSRS485, &rs485conf);
	if (ret < 0) {
		perror("unable to set rs485 conf (%m)");
                exit(-1);
        }

	/* Set up the serial communication attributes */
        memset(&term, 0, sizeof(term));
	ret = tcgetattr(fd, &term);
	if (ret < 0) {
		perror("unable to get term attributes");
		exit(-1);
	}

	ret = cfsetispeed(&term, B115200);
	if (ret < 0) {
		perror("unable to set input speed");
		exit(-1);
   	}
	ret = cfsetospeed(&term, B115200);
	if (ret < 0) {
		perror("unable to set output speed");
		exit(-1);
	}

	cfmakeraw(&term);
	term.c_cc[VTIME] = 0;
	term.c_cc[VMIN] = 1;
	ret = tcsetattr(fd, TCSADRAIN, &term);
	if (ret < 0) {
		perror("unable to set term attributes");
		exit(-1);
	}

	/* Transmission: enable parity multidrop and mark 1st byte as address */
	term.c_cflag |= PARENB | CMSPAR | PARMD | SENDA;
	/* Reception: enable parity multidrop and parity check */ 
	term.c_iflag |= PARENB | PARMD | INPCK;
	ret = tcsetattr(fd, TCSADRAIN, &term);                     
	if (ret < 0) {
		perror("unable to set MARK");
		exit(-1);
	}

	/* Write data */
	n = 0;
	buf[n++] = 0x00;
	buf[n++] = 0x1c;
	buf[n++] = 0x01;
	buf[n++] = 0x01;
	buf[n++] = 0x00;
	buf[n++] = 0x01;
	buf[n++] = 0xde;
	buf[n++] = 0xd4;
        ret = writen(fd, buf, n);      
        if (ret < 0) {
                perror("unable to write data (%m)");
                exit(-1);
        }

	//tcflush(fd, TCIOFLUSH);

	/* Reception */
	ret = readn(fd, buf, 3);   
        if (ret < 0) {
                perror("unable to read data1 (%m)");
                exit(-1);
        }
	for (i = 0; i < 3; i++)
		printf("%02x ", buf[i]);
	printf("\n");

	n = buf[1] + 2;
	ret = readn(fd, buf, n);   
        if (ret < 0) {
                perror("unable to read data2 (%m)");
                exit(-1);
        }
	for (i = 0; i < n; i++)
		printf("%02x ", buf[i]);
	printf("\n");

	return 0;
}

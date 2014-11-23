#include "SerialThread.h"

#include <fcntl.h>
#include <stdio.h>
#include <errno.h>

#define PACKET_SIZE_BYTES 5

SerialThread::SerialThread()
{
	connected = false;
}

bool SerialThread::connect(const QString &sDevice)
{
	const char* device = sDevice.toLatin1().data();
	if ((fd = open (device, O_RDONLY | O_NOCTTY)) < 0) {
		emit statusMessage(QString("Connection failed: ") + strerror(errno), 2000);
		return false;
	}
	emit statusMessage("Successfully connected.", 2000);

	tcgetattr(fd,&oldtio); /* save current port settings */

	bzero(&newtio, sizeof(newtio));
	newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
	newtio.c_cc[VMIN]     = PACKET_SIZE_BYTES;   /* blocking read until 5 chars received */

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd,TCSANOW,&newtio);

	start();

	return (connected = true);
}

bool SerialThread::disconnect()
{
	if (!connected) return false;

	connected = false;
	wait();

	tcsetattr(fd,TCSANOW,&oldtio);
	close(fd);

	emit statusMessage("Disconnected.", 2000);
	return true;
}

void SerialThread::setUpdateFunction(const SerialThread::UpdateFunction &f)
{
	updateFunc = f;
}

void SerialThread::run()
{
	int res;
	unsigned short frame[NO_TAXELS];
	unsigned char  buf[PACKET_SIZE_BYTES]; // receive buffer
	unsigned char  index;
	bzero(frame, sizeof(frame));

	fd_set fdset;
	struct timespec timeout;
	FD_ZERO (&fdset);
	FD_SET (fd,&fdset);
	timeout.tv_sec = 0;
	timeout.tv_nsec = 100e6; // 100 ms

	while (connected) {
		if (-1 == (res = pselect (fd+1,&fdset,NULL,NULL,&timeout,NULL))) {
			perror ("select");
			exit (EXIT_FAILURE);
		}

		if (res > 0) {
			res = read(fd,buf,5);   // read a maximum of 5 bytes into buf (actual read count is in res)
/* Parsing:
   we go through the res bytes in buf[] and look for
   * first byte determines taxel number, allowed range is from 0x3C to 0x7B;
   * constant byte 0x01
   * 2 byte sensor value, with first 4 bits as internal AD channel number (ignore)
   * NULL byte (0x00)
   Example: 3C 01 0F FF 00
*/

			if (res == 5) {
				if((buf[0]>=0x3C) && ((index = buf[0] - 0x3C) < NO_TAXELS) &&
				      buf[1] == 0x01 && buf[4] == 0x00) {
					// We have a full packet
					unsigned short value = ((0x0F & buf[2])<<8) | buf[3]; // Get pressure value
					frame[index] = 4095-value;

					// got full frame ?
					if(index==NO_TAXELS-1)
						updateFunc(frame);
				}
			}
		}
	}
}

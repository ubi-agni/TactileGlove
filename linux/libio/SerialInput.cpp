/* ============================================================
 *
 * Copyright (C) 2015 by Robert Haschke <rhaschke at techfak dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the "LGPL"),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CITEC, "Cognitive Interaction Technology" Excellence Cluster
 *     Bielefeld University
 *
 * ============================================================ */
#include "SerialInput.h"

#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

using namespace std;

namespace tactile {

static const size_t PACKET_SIZE_BYTES = 5;

const char *SerialInput::timeout_error::what() const throw()
{
	return "serial communication timed out";
}

SerialInput::SerialInput(size_t noTaxels) : InputInterface(noTaxels)
{
	setTimeOut(100);
}
SerialInput::~SerialInput()
{
	SerialInput::disconnect();
}

void SerialInput::setTimeOut(unsigned int msec)
{
	timeout.tv_sec = 0;
	timeout.tv_nsec = msec * 1e6;
}

void SerialInput::connect(const std::string &sDevice)
{
	if (connected)
		return;
	if ((fd = open(sDevice.c_str(), O_RDONLY | O_NOCTTY)) < 0)
		throw std::runtime_error(string("Connection failed: ") + strerror(errno));

	tcgetattr(fd, &oldtio); /* save current port settings */
	memset(&newtio, 0, sizeof(newtio));

	cfsetospeed(&newtio, B115200);
	cfsetispeed(&newtio, B115200);

	// set up raw mode / no echo / binary
	newtio.c_cflag |= (tcflag_t)(CLOCAL | CREAD);
	newtio.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);  //|ECHOPRT

	newtio.c_oflag &= (tcflag_t) ~(OPOST);
	newtio.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);
#ifdef IUCLC
	newtio.c_iflag &= (tcflag_t)~IUCLC;
#endif
#ifdef PARMRK
	newtio.c_iflag &= (tcflag_t)~PARMRK;
#endif

	// setup char len = CS8
	newtio.c_cflag |= CS8;
	// setup one stopbit
	newtio.c_cflag &= (tcflag_t) ~(CSTOPB);
	// setup parity: no parity
	newtio.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
	newtio.c_cflag &= (tcflag_t) ~(PARENB | PARODD);
	// setup flow control: none
	newtio.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
	newtio.c_cflag &= (tcflag_t) ~(CRTSCTS);

	newtio.c_cc[VTIME] = 0; /* inter-character timer unused */
	newtio.c_cc[VMIN] = 0; /* no minimum chars to be read */

	tcsetattr(fd, TCSANOW, &newtio);
	tcflush(fd, TCIFLUSH);

	connected = true;
	readFrame();  // read eventually incomplete frame
}

void SerialInput::disconnect()
{
	if (!connected)
		return;
	connected = false;
	tcsetattr(fd, TCSANOW, &oldtio);
	tcflush(fd, TCIOFLUSH);
	close(fd);
}

const InputInterface::data_vector &SerialInput::readFrame()
{
	if (!connected)
		throw std::runtime_error("not connected");

	unsigned char buf[PACKET_SIZE_BYTES];  // receive buffer
	size_t index;
	fd_set fdset;

	while (connected) {
		FD_ZERO(&fdset);
		FD_SET(fd, &fdset);
		int res = pselect(fd + 1, &fdset, nullptr, nullptr, &timeout, nullptr);
		if (res == -1)
			throw std::runtime_error(strerror(errno));
		if (res == 0)
			throw timeout_error();

		// read a maximum of 5 bytes into buf (actual read count is in res)
		res = read(fd, buf, PACKET_SIZE_BYTES);

		/* Parsing: We go through the res bytes in buf[] and look for
		   - first byte determines taxel number, allowed range is from 0x3C to 0x7B;
		   - constant byte 0x01
		   - 2 byte sensor value, with first 4 bits as internal AD channel number (ignore)
		   - NULL byte (0x00)
		   Example: 3C 01 0F FF 00
		*/
		if (res == PACKET_SIZE_BYTES) {
			if ((buf[0] >= 0x3C) && ((index = buf[0] - 0x3C) < data.size()) && buf[1] == 0x01 && buf[4] == 0x00) {
				// we have a valid packet
				data_type value = ((0x0F & buf[2]) << 8) | buf[3];  // get pressure value
				data[index] = 4095 - value;
				// got full frame ?
				if (index == data.size() - 1)
					return data;
			} else
				sync(buf);
		} else
			throw std::runtime_error("failed to read from serial input");
	}

	return data;
}

inline bool valid(const unsigned char buf[PACKET_SIZE_BYTES], unsigned int o)
{
	return buf[o % PACKET_SIZE_BYTES] >= 0x3C && buf[(1 + o) % PACKET_SIZE_BYTES] == 0x01 &&
	       buf[(4 + o) % PACKET_SIZE_BYTES] == 0x00;
}

void SerialInput::sync(unsigned char buf[PACKET_SIZE_BYTES]) const
{
	unsigned int offset = 1;
	for (; offset < PACKET_SIZE_BYTES; ++offset)
		if (valid(buf, offset))
			break;
	read(fd, buf, PACKET_SIZE_BYTES - offset);
}

}  // namespace tactile

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


SerialInput::SerialInput(size_t noTaxels)
   : InputInterface(noTaxels)
{
	setTimeOut(100);
}
SerialInput::~SerialInput() {
	disconnect();
}

void SerialInput::setTimeOut(unsigned int msec)
{
	timeout.tv_sec = 0;
	timeout.tv_nsec = msec * 1e6;
}

void SerialInput::connect(const std::string &sDevice)
{
	if (connected) return;
	if ((fd = open (sDevice.c_str(), O_RDONLY | O_NOCTTY)) < 0)
		throw std::runtime_error(string("Connection failed: ") + strerror(errno));

	tcgetattr(fd,&oldtio); /* save current port settings */

	bzero(&newtio, sizeof(newtio));
	newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
	newtio.c_cc[VMIN]     = PACKET_SIZE_BYTES;   /* blocking read until 5 chars received */

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd,TCSANOW,&newtio);

	FD_ZERO (&fdset);
	FD_SET (fd,&fdset);

	connected = true;
	readFrame(); // read eventually incomplete frame
}

void SerialInput::disconnect()
{
	if (!connected) return;
	connected = false;
	tcsetattr(fd,TCSANOW,&oldtio);
	close(fd);
}

const InputInterface::data_vector& SerialInput::readFrame()
{
	if (!connected) throw std::runtime_error("not connected");

	unsigned char buf[PACKET_SIZE_BYTES]; // receive buffer
	size_t index;

	while (connected) {
		int res = pselect (fd+1,&fdset,NULL,NULL,&timeout,NULL);
		if (res == -1) throw std::runtime_error(strerror(errno));
		if (res == 0) 	throw timeout_error();

		// read a maximum of 5 bytes into buf (actual read count is in res)
		res = read(fd,buf,PACKET_SIZE_BYTES);

		/* Parsing: We go through the res bytes in buf[] and look for
			- first byte determines taxel number, allowed range is from 0x3C to 0x7B;
			- constant byte 0x01
			- 2 byte sensor value, with first 4 bits as internal AD channel number (ignore)
			- NULL byte (0x00)
			Example: 3C 01 0F FF 00
		*/
		if (res == PACKET_SIZE_BYTES) {
			if ((buf[0]>=0x3C) && ((index = buf[0] - 0x3C) < data.size()) &&
			    buf[1] == 0x01 && buf[4] == 0x00) {
				// we have a valid packet
				data_type value = ((0x0F & buf[2])<<8) | buf[3]; // get pressure value
				data[index] = 4095-value;
				// got full frame ?
				if (index==data.size()-1) return data;
			}
		} else throw std::runtime_error ("failed to read from serial input");
	}

	return data;
}

}

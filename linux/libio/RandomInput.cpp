/* ============================================================
 *
 * Copyright (C) 2014 by Robert Haschke <rhaschke at techfak dot uni-bielefeld dot de>
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

#include "RandomInput.h"
#include "TimeUtils.h"
#include <stdlib.h>
#include <stdexcept>
#include <unistd.h>

namespace tactile {

RandomInput::RandomInput(size_t noTaxels, const uint64_t period)
   : InputInterface(noTaxels),
  period(period)
{
	// initialize smallest values
	data[1] = 2048;
	data[14] = 3700;
}

void RandomInput::connect(const std::string &dummy)
{
	connected = true;
	prev_time = getMicroseconds();
}

void RandomInput::disconnect()
{
	connected = false;
}

const InputInterface::data_vector& RandomInput::readFrame()
{
	if (!connected) throw std::runtime_error("not connected");
	
	// first prepare data (takes some time)
	size_t idx = 0;
	for (data_vector::iterator it=data.begin(), end=data.end(); it != end; ++it, ++idx) {
		long int rndnumber = random();
		if (rndnumber < (RAND_MAX / 2)) { /* only give new value 50% of time */
			*it = rndnumber & 0xFFF;
			if (idx != 0) *it /= 10;   // except of first value, all have a rather small stddev
			if (idx == 1)  *it = std::min<data_type>(*it + 2048, 0xFFF); // simulate a fixed offset
			if (idx == 14) *it = std::min<data_type>(*it + 3700, 0xFFF); // goniometer joint value
		}
	}
	// then check time elapsed
	uint64_t cur_time = getMicroseconds();
	uint64_t time_diff_us = cur_time - prev_time;
	//printf("time us %ld\n", time_diff_us);

	// if required wait until period has passed
	if (time_diff_us < period)
	{
		usleep(period - time_diff_us);
	}

	prev_time = getMicroseconds();  // store time when returning the data
	return data;
}

}

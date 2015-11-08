/* ============================================================
 *
 * Copyright (C) 2015 by Guillaume Walck <gwalck at techfak dot uni-bielefeld dot de>
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

#include "FileInput.h"
#include <stdlib.h>
#include <stdexcept>
#include <unistd.h>

namespace tactile {

FileInput::FileInput(size_t noTaxels, double factor, bool startover)
   : InputInterface(noTaxels), loop(startover)
{
	// initialize smallest values
	data[1] = 2048;
	data[14] = 3700;
	
	if(factor > 0)
		speed_factor = factor;
	else
		speed_factor = 1.0;
}

void FileInput::connect(const std::string &filename)
{
	fd = fopen(filename.c_str(), "r");
	char buf[256];
	if (fd == NULL)
		throw std::runtime_error("cannot open file");
	else
	{
		// jump header
		while (fgets(buf, sizeof buf, fd)) {
			if (*buf == '#') continue; /* ignore comment line */
			if (*buf == '\n') break;
		}
		if (*buf == '\n')
		{
			fgetpos(fd, &startpos);
			connected = true;
			prev_timestamp = 0;
			time(&prev_time);
		}
	}
}

void FileInput::disconnect()
{
	if (fd != NULL) {
		fclose(fd);
		fd = NULL;
	}
	connected = false;
}

const FileInput::data_vector& FileInput::readFrame()
{
	if (!connected) throw std::runtime_error("not connected");
	time_t cur_time;
	time(&cur_time);
	double time_diff_ms = difftime(cur_time, prev_time)* 1000.;
	prev_time = cur_time;
	
	size_t idx = 0;
	int timestamp;
	char buf[256];
	int comment_chars = 1;
	short unsigned int val;
	if (feof(fd))
	{
		if (loop) {
			fsetpos(fd, &startpos);
			prev_timestamp = 0;
			printf("loop\n");
		}
		else {
			throw std::runtime_error("end of data");
		}
	}
	if (!feof(fd))
	{
		// jump comment lines
		while(comment_chars && !feof(fd)) {
			comment_chars = fscanf(fd,"# %s\n", buf);
		}

		// read timestamp
		if (fscanf(fd,"%d", &timestamp) == 1)
		{
			if (prev_timestamp != 0)
			{
				double diff_timestamp = (timestamp - prev_timestamp);
				if (diff_timestamp >= 0 && diff_timestamp < 2000)
				{
					if (time_diff_ms < diff_timestamp / speed_factor)
						usleep(1000 * diff_timestamp / speed_factor);
					prev_timestamp = timestamp;
				}
				else
				{
					throw std::runtime_error("timestamps must be increasing and less than 2 seconds appart");
				}
			}
			else
			{
				prev_timestamp = timestamp;
			}

			for (data_vector::iterator it=data.begin(), end=data.end(); it != end; ++it, ++idx) {
				if(fscanf(fd,";%hu", &val) != 1) {
					if(!feof(fd))
						fscanf(fd,"\n");
					break;
				}
				else {
					*it = val;
				}
			}
		}
	}
	return data;
}

}

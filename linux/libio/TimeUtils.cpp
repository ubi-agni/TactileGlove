/* ============================================================
 *
 * Copyright (C) 2022 by Guillaume Walck <gwalck at techfak dot uni-bielefeld dot de>
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
#include "TimeUtils.h"

/// Convert seconds to microseconds
#define SEC_TO_US(sec) ((sec) * 1000000) 

uint64_t getMicroseconds()
{
	uint64_t microseconds;
	struct timespec ts;
	int return_code = timespec_get(&ts, TIME_UTC);
	if (return_code == 0)
	{
		printf("Failed to obtain timestamp.\n");
		microseconds = UINT64_MAX; // use this to indicate error
	}
	else
	{
		// `ts` now contains your timestamp in seconds and nanoseconds! To 
		// convert the whole struct to nanoseconds, do this:
		microseconds = SEC_TO_US((uint64_t)ts.tv_sec) + ((uint64_t)ts.tv_nsec) / 1000;
	}
	return microseconds;
}

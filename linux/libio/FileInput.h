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
#pragma once

#include "InputInterface.h"
#include <stdio.h>
#include <time.h>

namespace tactile {

class FileInput : public InputInterface
{
public:
	FileInput(size_t noTaxels, double speed_factor = 1.0, bool startover = false);
	void connect(const std::string &dummy) override;
	void disconnect() override;
	const data_vector &readFrame() override;

private:
	FILE *fd;
	bool loop;
	fpos_t startpos;
	time_t prev_time;
	int prev_timestamp;
	float speed_factor;
};

}  // namespace tactile

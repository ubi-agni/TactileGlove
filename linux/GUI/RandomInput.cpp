/* ============================================================
 *
 * This file is a part of the RSB project
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
#include <QTimerEvent>

RandomInput::RandomInput(size_t noTaxels) : input(noTaxels), timerID(0) {}

bool RandomInput::connect(const QString & /*input*/)
{
	input.connect("");
	timerID = startTimer(100);
	return input.isConnected();
}

bool RandomInput::disconnect()
{
	input.disconnect();
	killTimer(timerID);
	timerID = 0;
	return input.isConnected();
}

void RandomInput::timerEvent(QTimerEvent *event)
{
	if (event->timerId() != timerID)
		return;
	updateFunc(input.readFrame());
}

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
#pragma once

#include "InputInterface.h"
#include "../libio/RandomInput.h"
#include <QObject>
#include <boost/function.hpp>

class RandomInput : public QObject, public InputInterface
{
	Q_OBJECT
signals:
	void statusMessage(const QString &message, int time);

public:
	RandomInput(size_t noTaxels);
	bool connect(const QString &dummy) override;
	bool disconnect() override;

	using data_vector = tactile::InputInterface::data_vector;
	using UpdateFunction = boost::function<void(const data_vector &)>;
	void setUpdateFunction(const UpdateFunction &f) { updateFunc = f; }

private:
	tactile::RandomInput input;
	UpdateFunction updateFunc;
	void timerEvent(QTimerEvent *event) override;
	int timerID;
};

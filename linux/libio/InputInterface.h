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
#pragma once
#include <string>
#include <vector>

namespace tactile {

class InputInterface
{
public:
	using data_type = unsigned short;
	using data_vector = std::vector<data_type>;

	explicit InputInterface(size_t noTaxels);
	virtual ~InputInterface();

	virtual void connect(const std::string& sInput) = 0;
	virtual void disconnect() = 0;
	virtual bool isConnected() const;

	virtual const data_vector& readFrame() = 0;

protected:
	data_vector data;
	bool connected;
};

}  // namespace tactile

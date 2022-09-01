/* ============================================================
 *
 * Copyright (C) 2022 by Robert Haschke, Bielefeld University
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
 * ============================================================ */
#pragma once

#include "Rate.h"

namespace tactile {

template <typename T>
class ThrottledInput : public T
{
public:
	using T::T;  // inherit all constructors
	const typename T::data_vector &readFrame() override
	{
		rate_.sleep();
		return T::readFrame();
	}

	const Rate &rate() const { return rate_; }
	void setRate(const Rate &rate) { rate_ = rate; }

private:
	Rate rate_ = 1000.0;  // default: 1kHz
};

}  // namespace tactile

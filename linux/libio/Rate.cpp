/*
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
#include "Rate.h"
#include <thread>

Rate::Rate(double frequency)
    : start_(Clock::now()), expected_cycle_time_(static_cast<Duration::rep>(Duration::period::den / Duration::period::num / frequency)), actual_cycle_time_(Clock::duration::zero())
{
}

Rate::Rate(const Duration &d)
    : start_(Clock::now()), expected_cycle_time_(d), actual_cycle_time_(Clock::duration::zero())
{
}

bool Rate::sleep()
{
  Time expected_end = start_ + expected_cycle_time_;
  Time actual_end = Clock::now();

  // detect backward jumps in time
  if (actual_end < start_)
    expected_end = actual_end + expected_cycle_time_;

  // calculate the time we'll sleep for
  Duration sleep_time = expected_end - actual_end;

  // set the actual amount of time the loop took in case the user wants to know
  actual_cycle_time_ = actual_end - start_;

  // make sure to reset our start time
  start_ = expected_end;

  // if we've taken too much time, we won't sleep
  if (sleep_time <= Duration::zero())
  {
    // if we've jumped forward in time, or the loop has taken more than a full extra
    // cycle, reset our cycle
    if (actual_end > expected_end + expected_cycle_time_)
      start_ = actual_end;

    // return false to show that the desired rate was not met
    return false;
  }

  std::this_thread::sleep_for(sleep_time);
  return true;
}

void Rate::reset()
{
  start_ = Clock::now();
}

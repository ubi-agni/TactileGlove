/*
 * Copyright (C) 2022 by Robert Haschke, Bielefeld University
 * This code was inspired by ros::Rate from https://github.com/ros/roscpp_core
 * Copyright (c) 2009, Willow Garage, Inc.
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
#include <chrono>

class Rate
{
public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  using Duration = Clock::duration;

  Rate(double frequency);
  explicit Rate(const Duration &);

  bool sleep();
  void reset();

  Duration cycleTime() const { return actual_cycle_time_; }

  Duration expectedCycleTime() const { return expected_cycle_time_; }

private:
  Time start_;
  Duration expected_cycle_time_, actual_cycle_time_;
};

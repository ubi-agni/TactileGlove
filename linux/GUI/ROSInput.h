#pragma once

#include "InputInterface.h"
#include <QObject>
#include <boost/function.hpp>
#include <ros/ros.h>
#include <tactile_msgs/TactileState.h>

namespace ros {
class Subscriber;
}

class ROSInput : public QObject, public InputInterface
{
	Q_OBJECT
signals:
	void statusMessage(const QString &, int time);

public:
	ROSInput(size_t noTaxels);
	~ROSInput();
	bool connect(const QString &sTopic);
	bool disconnect();

	typedef boost::function<void(const std::vector<float> &)> UpdateFunction;
	void setUpdateFunction(const UpdateFunction &f) { updateFunc = f; }

private:
	void receiveCallback(const tactile_msgs::TactileState &msg);

private:
	UpdateFunction updateFunc;
	size_t noTaxels;
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner;
	ros::Subscriber subscriber;
};

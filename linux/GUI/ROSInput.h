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
	void statusMessage(const QString &message, int time);

public:
	ROSInput(size_t noTaxels);
	~ROSInput() override;
	bool connect(const QString &sTopic) override;
	bool disconnect() override;

	using UpdateFunction = boost::function<void(const std::vector<float> &)>;
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

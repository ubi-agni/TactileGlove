#pragma once

#include "InputInterface.h"
#include <QObject>
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>

namespace ros {class Subscriber;}

class ROSInput : public QObject, public InputInterface
{
	Q_OBJECT
signals:
	void statusMessage(const QString&, int time);

public:
	ROSInput(size_t noTaxels);
	~ROSInput();
	bool connect(const QString &sTopic);
	bool disconnect();

private:
	void receiveCallback(const std_msgs::UInt16MultiArray& msg);

private:
	size_t            noTaxels;
	ros::NodeHandle   nh;
	ros::AsyncSpinner spinner;
	ros::Subscriber   subscriber;
};

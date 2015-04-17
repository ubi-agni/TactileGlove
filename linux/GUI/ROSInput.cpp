#include "ROSInput.h"

ROSInput::ROSInput(size_t noTaxels) :
   noTaxels(noTaxels), spinner(1)
{
}

ROSInput::~ROSInput()
{
	spinner.stop();
	subscriber.shutdown();
}

bool ROSInput::connect(const QString &sTopic)
{
	try {
		subscriber = nh.subscribe(sTopic.toStdString(), 10, &ROSInput::receiveCallback, this);
		spinner.start();
		return true;
	} catch (const ros::InvalidNameException &e) {
		emit statusMessage(QString("failed to create subscriber ").append(sTopic), 2000);
	} catch (const std::exception &e) {
		const std::string &resolved = nh.resolveName(sTopic.toStdString());
		emit statusMessage(QString("failed to create subscriber ").append(resolved.c_str()), 2000);
	}
	return false;
}

bool ROSInput::disconnect()
{
	subscriber.shutdown();
	spinner.stop();
	return true;
}

void ROSInput::receiveCallback(const std_msgs::UInt16MultiArray &msg)
{
	assert(msg.layout.dim[0].size == 64);
	updateFunc(msg.data);
}

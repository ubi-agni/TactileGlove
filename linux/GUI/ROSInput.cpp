#include "ROSInput.h"
#include "GloveWidget.h"
#include <QDebug>

ROSInput::ROSInput() :
   spinner(1)
{
}

ROSInput::~ROSInput()
{
	spinner.stop();
	subscriber.shutdown();
}

bool ROSInput::connect(const QString &sPublisher)
{
	subscriber = nh.subscribe(sPublisher.toStdString(), 10, &ROSInput::receiveCallback, this);
	spinner.start();
	return true;
}

bool ROSInput::disconnect()
{
	subscriber.shutdown();
	spinner.stop();
	return true;
}

void ROSInput::receiveCallback(const std_msgs::UInt16MultiArray &msg)
{
	assert(msg.layout.dim[0].size == NO_TAXELS);
	updateFunc(msg.data.data());
}

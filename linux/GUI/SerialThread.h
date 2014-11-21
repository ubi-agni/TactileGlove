#pragma once

#include "GloveWidget.h"

#include <QThread>
#include <boost/function.hpp>
#include <termios.h>

class SerialThread : public QThread
{
	Q_OBJECT
public:
	typedef boost::function<void(unsigned short*)> UpdateFunction;

	SerialThread();
	bool connect(const char* device);
	bool disconnect();
	void setUpdateFunction(const UpdateFunction &f);

protected:
	void run();

private:
	struct termios oldtio,newtio;
	int fd;
	UpdateFunction updateFunc;

	bool connected;
};

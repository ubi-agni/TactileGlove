#pragma once

#include "InputInterface.h"

#include <QThread>
#include <termios.h>

class SerialThread : public QThread, public InputInterface
{
	Q_OBJECT
signals:
	void statusMessage(const QString&, int time);

public:
	SerialThread();
	bool connect(const QString &sDevice);
	bool disconnect();

protected:
	void run();

private:
	struct termios oldtio,newtio;
	int fd;

	bool connected;
};

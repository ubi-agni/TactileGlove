#pragma once

#include "InputInterface.h"
#include "../lib/SerialInput.h"
#include <QThread>

class SerialThread : public QThread, public InputInterface
{
	Q_OBJECT
signals:
	void statusMessage(const QString&, int time);

public:
	SerialThread(size_t noTaxels);
	bool connect(const QString &sDevice);
	bool disconnect();

protected:
	void run();

private:
	tactile::SerialInput input;
};

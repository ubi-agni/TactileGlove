#pragma once

#include "InputInterface.h"
#include "GloveWidget.h"
#include <QObject>

class RandomInput : public QObject, public InputInterface
{
	Q_OBJECT
signals:
	void statusMessage(const QString&, int time);

public:
	RandomInput();
	bool connect(const QString &sPublisher);
	bool disconnect();

private:
	unsigned short data[NO_TAXELS];
	void timerEvent(QTimerEvent *event);

	int timerID;
};

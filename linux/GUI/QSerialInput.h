#pragma once

#include "InputInterface.h"
#include "../libio/InputInterface.h"
#include <qobject.h>
#include <boost/function.hpp>

class QSerialPort;
class QSerialInput : public QObject, public InputInterface
{
	Q_OBJECT

signals:
	void statusMessage(const QString &message, int time);
	void disconnected(const QString &reason);

private:
	void sync(unsigned char buf[]) const;

public:
	QSerialInput(size_t noTaxels);
	bool connect(const QString &sDevice) Q_DECL_OVERRIDE;
	bool disconnect() Q_DECL_OVERRIDE;

	using data_vector = tactile::InputInterface::data_vector;
	using UpdateFunction = boost::function<void(const data_vector &)>;
	void setUpdateFunction(const UpdateFunction &f) { updateFunc = f; }

private slots:
	void readData();
	void onError();

private:
	QSerialPort *serial;
	tactile::InputInterface::data_vector frame;
	UpdateFunction updateFunc;
};

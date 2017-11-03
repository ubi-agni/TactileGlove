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
	void statusMessage(const QString&, int time);
	void disconnected(const QString& reason);

private:
	void sync(unsigned char buf[]) const;

public:
	QSerialInput(size_t noTaxels);
	bool connect(const QString &sDevice) Q_DECL_OVERRIDE;
	bool disconnect() Q_DECL_OVERRIDE;

	typedef tactile::InputInterface::data_vector data_vector;
	typedef boost::function<void(const data_vector&)> UpdateFunction;
	void setUpdateFunction(const UpdateFunction &f) {updateFunc = f;}

private slots:
	void readData();
	void onError();

private:
	QSerialPort *serial;
	tactile::InputInterface::data_vector frame;
	UpdateFunction updateFunc;
};

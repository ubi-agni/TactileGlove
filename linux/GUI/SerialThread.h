#pragma once

#include "InputInterface.h"
#include "../libio/SerialInput.h"
#include <QThread>
#include <boost/function.hpp>

class SerialThread : public QThread, public InputInterface
{
	Q_OBJECT
signals:
	void statusMessage(const QString&, int time);

public:
	SerialThread(size_t noTaxels);
	bool connect(const QString &sDevice);
	bool disconnect();

	typedef tactile::InputInterface::data_vector data_vector;
	typedef boost::function<void(const data_vector&)> UpdateFunction;
	void setUpdateFunction(const UpdateFunction &f) {updateFunc = f;}

protected:
	void run();

private:
	tactile::SerialInput input;
	UpdateFunction updateFunc;
};

#include "QSerialInput.h"
#include <qtimer.h>
#include <qdebug.h>
#include <QtSerialPort/qserialport.h>

QSerialInput::QSerialInput(size_t noTaxels) : serial(nullptr), frame(noTaxels)
{
	serial = new QSerialPort(this);
	serial->setFlowControl(QSerialPort::NoFlowControl);
	serial->setParity(QSerialPort::NoParity);
	serial->setStopBits(QSerialPort::OneStop);
	serial->setDataBits(QSerialPort::Data8);
	serial->setBaudRate(QSerialPort::Baud115200);

	QObject::connect(serial, SIGNAL(readyRead()), this, SLOT(readData()));
	QObject::connect(serial, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(onError()));
}

bool QSerialInput::connect(const QString &sDevice)
{
	if (serial->isOpen())
		return true;

	serial->setPortName(sDevice);
	if (!serial->open(QIODevice::ReadOnly))
		emit statusMessage(serial->errorString(), 2000);

	return serial->isOpen();
}

bool QSerialInput::disconnect()
{
	if (!serial->isOpen())
		return true;
	serial->close();
	Q_EMIT disconnected(QString());
	return true;
}

void QSerialInput::onError()
{
	switch (serial->error()) {
	case QSerialPort::NoError:
		return;
	case QSerialPort::ResourceError:
		Q_EMIT disconnected(tr("Lost connection"));
		// serial->close(); // crashes in QSerialPort lib
		break;
	default:
		Q_EMIT disconnected(serial->errorString());
		break;
	}
}

static const size_t PACKET_SIZE_BYTES = 5;

void QSerialInput::readData()
{
	unsigned char buf[PACKET_SIZE_BYTES];
	size_t index, max_index = frame.size() - 1;

	while (static_cast<size_t>(serial->bytesAvailable()) >= PACKET_SIZE_BYTES) {
		const auto bytes = serial->read((char *)buf, PACKET_SIZE_BYTES);
		Q_ASSERT(bytes == PACKET_SIZE_BYTES);
		Q_UNUSED(bytes);

		if ((buf[0] >= 0x3C) && ((index = buf[0] - 0x3C) <= max_index) && buf[1] == 0x01 && buf[4] == 0x00) {
			// we have a valid packet
			tactile::InputInterface::data_type value = ((0x0F & buf[2]) << 8) | buf[3];  // extract value
			frame[index] = 4095 - value;

			// got full frame ?
			if (index == max_index)
				updateFunc(frame);
		} else
			sync(buf);
	}
}

inline bool valid(const unsigned char buf[PACKET_SIZE_BYTES], unsigned int o)
{
	return buf[o % PACKET_SIZE_BYTES] >= 0x3C && buf[(1 + o) % PACKET_SIZE_BYTES] == 0x01 &&
	       buf[(4 + o) % PACKET_SIZE_BYTES] == 0x00;
}

void QSerialInput::sync(unsigned char buf[PACKET_SIZE_BYTES]) const
{
	unsigned int offset = 1;
	for (; offset < PACKET_SIZE_BYTES; ++offset)
		if (valid(buf, offset))
			break;
	serial->read((char *)buf, PACKET_SIZE_BYTES - offset);
}

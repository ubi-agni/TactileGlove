#include "MainWindow.h"
#include "ui_MainWindow.h"

#include "SerialThread.h"
#include "RandomInput.h"
#if HAVE_ROS
#include "ROSInput.h"
#endif

#include <boost/bind.hpp>

MainWindow::MainWindow(QWidget *parent) :
   QMainWindow(parent), ui(new Ui::MainWindow),
   input(0), frameCount(0), timerID(0)
{
	bzero(frameData, sizeof(frameData));

	ui->setupUi(this);
	ui->toolBar->addWidget(ui->updateTimeSpinBox);
	ui->toolBar->addWidget(ui->lambdaSpinBox);
	ui->toolBar->addWidget(ui->inputLineEdit);
	ui->toolBar->addWidget(ui->btnConnect);
	ui->toolBar->addWidget(ui->btnDisconnect);
	ui->btnDisconnect->setEnabled(false);
	ui->fps->hide();

	ui->verticalLayout->insertWidget(0, gloveWidget = new GloveWidget);

	connect(ui->updateTimeSpinBox, SIGNAL(valueChanged(int)), this, SLOT(setTimer(int)));
	connect(ui->lambdaSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setLambda(double)));
}

MainWindow::~MainWindow()
{
	if (input) {
		input->disconnect();
		delete input;
	}
	delete ui;
}

void MainWindow::setTimer(int interval)
{
	if (!timerID) return;
	killTimer(timerID);
	timerID = startTimer(interval);
}

void MainWindow::setLambda(double value)
{
	lambda = value;
}

void MainWindow::timerEvent(QTimerEvent *event)
{
	if (event->timerId() != timerID) return;

	dataMutex.lock();
	unsigned short data[NO_TAXELS];
	int fps = -1;
	std::copy(frameData, frameData+NO_TAXELS, data);
	if (lastUpdate.elapsed() > 1000) { // update framerate every 1s
		fps = roundf (float(frameCount * 1000) / lastUpdate.restart());
		frameCount = 0;
	}
	dataMutex.unlock();

	gloveWidget->update_data(data);
	updateJointBar(data[14]);
	if (fps >= 0) ui->fps->setText(QString("%1 fps").arg(fps));
}


void MainWindow::updateData(const unsigned short *data) {
	dataMutex.lock();

	float alpha = 1.0 - lambda;
	for (int i=0; i < NO_TAXELS; ++i)
		frameData[i] = lambda * frameData[i] + alpha * data[i];
	++frameCount;

	dataMutex.unlock();
}

void MainWindow::updateJointBar(unsigned short value)
{
	const int min=4095;
	const int max=2000;
	const int targetRange=100;
	ui->jointBar->setValue(((value-min) * targetRange) / (max-min));
}


void MainWindow::configSerial(const QString &sDevice)
{
	ui->inputLineEdit->setText(sDevice);
	ui->inputLineEdit->setToolTip("serial device name");

	SerialThread *serial = new SerialThread;
	serial->setUpdateFunction(boost::bind(&MainWindow::updateData, this, _1));
	connect(serial, SIGNAL(statusMessage(QString,int)),
	        ui->statusBar, SLOT(showMessage(QString,int)));
	input = serial;
}

void MainWindow::configROS(const QString &sTopic)
{
#if HAVE_ROS
	ui->inputLineEdit->setText(sTopic);
	ui->inputLineEdit->setToolTip("ROS topic");

	ROSInput *rosInput = new ROSInput;
	rosInput->setUpdateFunction(boost::bind(&MainWindow::updateData, this, _1));
	connect(rosInput, SIGNAL(statusMessage(QString,int)),
	        ui->statusBar, SLOT(showMessage(QString,int)));
	input = rosInput;

	on_btnConnect_clicked(); // auto-connect to ROS topic
#endif
}

void MainWindow::configRandom()
{
	ui->verticalLayout->addWidget(ui->inputLineEdit);
	ui->inputLineEdit->hide();
	input = new RandomInput;
	input->setUpdateFunction(boost::bind(&MainWindow::updateData, this, _1));
}

void MainWindow::on_btnConnect_clicked()
{
	ui->statusBar->showMessage (QString ("Connecting..."), 2000);

	if (input->connect(ui->inputLineEdit->text())) {
		frameCount = 0; lastUpdate.start();
		timerID = startTimer (ui->updateTimeSpinBox->value());

		ui->btnConnect->setEnabled(false);
		ui->btnDisconnect->setEnabled(true);
		ui->fps->show(); ui->toolBar->addWidget(ui->fps);
		ui->statusBar->showMessage("Successfully connected.", 2000);
	}
}

void MainWindow::on_btnDisconnect_clicked()
{
	ui->statusBar->showMessage("Disconnecting...",2000);
	ui->btnDisconnect->setEnabled(false);

	input->disconnect();
	gloveWidget->reset_data();
	ui->statusBar->showMessage("Disconnected.", 2000);

	ui->btnConnect->setEnabled(true);
	killTimer(timerID); timerID = 0;
}

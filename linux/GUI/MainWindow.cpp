#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "GloveWidget.h"
#include <boost/bind.hpp>

MainWindow::MainWindow(QWidget *parent) :
   QMainWindow(parent), ui(new Ui::MainWindow),
   frameCount(0), timerID(0)
{
	bzero(frameData, sizeof(frameData));

	ui->setupUi(this);
	ui->toolBar->addWidget(ui->updateTimeSpinBox);
	ui->toolBar->addWidget(ui->lambdaSpinBox);
	ui->toolBar->addWidget(ui->deviceLineEdit);
	ui->toolBar->addWidget(ui->btnConnect);
	ui->toolBar->addWidget(ui->btnDisconnect);
	ui->btnDisconnect->setEnabled(false);
	ui->fps->hide();

	ui->verticalLayout->insertWidget(0, gloveWidget = new GloveWidget);

	serialThread = new SerialThread;
	serialThread->setUpdateFunction(boost::bind(&MainWindow::updateData, this, _1));
	connect(serialThread, SIGNAL(statusMessage(QString,int)),
	        ui->statusBar, SLOT(showMessage(QString,int)));

	connect(ui->updateTimeSpinBox, SIGNAL(valueChanged(int)), this, SLOT(setTimer(int)));
	connect(ui->lambdaSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setLambda(double)));
}

MainWindow::~MainWindow()
{
	serialThread->disconnect();
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
		fps = frameCount * 1000 / lastUpdate.restart();
		frameCount = 0;
	}
	dataMutex.unlock();

	gloveWidget->update_data(data);
	updateJointBar(data[14]);
	if (fps >= 0) ui->fps->setText(QString("%1 fps").arg(fps));
}

void MainWindow::on_btnConnect_clicked()
{
	ui->statusBar->showMessage (QString ("Connecting..."), 2000);

	if (serialThread->connect(ui->deviceLineEdit->text())) {
		frameCount = 0; lastUpdate.start();
		timerID = startTimer (ui->updateTimeSpinBox->value());

		ui->btnConnect->setEnabled(false);
		ui->btnDisconnect->setEnabled(true);
		ui->fps->show(); ui->toolBar->addWidget(ui->fps);
	}
}

void MainWindow::on_btnDisconnect_clicked()
{
	ui->statusBar->showMessage("Disconnecting...",2000);
	ui->btnDisconnect->setEnabled(false);

	serialThread->disconnect();
	gloveWidget->reset_data();

	ui->btnConnect->setEnabled(true);
	killTimer(timerID); timerID = 0;
}

void MainWindow::updateData(unsigned short *data) {
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

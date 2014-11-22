#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "GloveWidget.h"
#include <boost/bind.hpp>

MainWindow::MainWindow(QWidget *parent) :
   QMainWindow(parent),
   ui(new Ui::MainWindow)
{
	lastUpdate.start(); frameCount = 0;

	ui->setupUi(this);
	ui->btnDisconnect->setEnabled(false);
	ui->serialToolBar->addWidget(ui->deviceLineEdit);
	ui->serialToolBar->addWidget(ui->btnConnect);
	ui->serialToolBar->addWidget(ui->btnDisconnect);
	ui->toolBar->addWidget(ui->updateTimeSpinBox);
	ui->fps->hide();

	ui->verticalLayout->insertWidget(0, gloveWidget = new GloveWidget);

	serialThread = new SerialThread;
	serialThread->setUpdateFunction(boost::bind(&MainWindow::updateData, this, _1));

	connect(this, SIGNAL(jointChanged(int)), ui->jointBar, SLOT(setValue(int)));
}

MainWindow::~MainWindow()
{
	serialThread->disconnect();
	delete ui;
}

void MainWindow::on_btnConnect_clicked()
{
	ui->statusBar->showMessage (QString ("Connecting..."), 2000);
	lastUpdate.restart();
	if (serialThread->connect(ui->deviceLineEdit->text()))
	{
		ui->statusBar->showMessage("Successfully connected!",2000);
		ui->btnConnect->setEnabled(false);
		ui->btnDisconnect->setEnabled(true);
		ui->fps->show(); ui->toolBar->addWidget(ui->fps);
	}
	else
	{
		ui->statusBar->showMessage("Connection failed!",2000);
	}
}

void MainWindow::on_btnDisconnect_clicked()
{
	ui->statusBar->showMessage("Disconnecting...",2000);
	ui->btnDisconnect->setEnabled(false);

	serialThread->disconnect();
	gloveWidget->reset_data();

	ui->statusBar->showMessage("Disconnected!",2000);
	ui->btnConnect->setEnabled(true);
}

void MainWindow::updateData(unsigned short *data) {
	// TODO data smoothing
	++frameCount;
	if (lastUpdate.elapsed() < ui->updateTimeSpinBox->value())
		return;

	gloveWidget->update_data(data);
	updateJointBar(data[14]);

	uint fps = frameCount * 1000 / lastUpdate.restart();
	ui->fps->setText(QString("%1 fps").arg(fps));
	frameCount = 0;
}

void MainWindow::updateJointBar(unsigned short data)
{
	const int min=4095;
	const int max=2000;
	const int targetRange=100;
	emit jointChanged(((data-min) * targetRange) / (max-min));
}

#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "GloveWidget.h"
#include <boost/bind.hpp>

MainWindow::MainWindow(QWidget *parent) :
   QMainWindow(parent),
   ui(new Ui::MainWindow)
{
	lastUpdate.start();

	ui->setupUi(this);
	ui->verticalLayout->insertWidget(1, gloveWidget = new GloveWidget);
	ui->btnDisconnect->setEnabled(false);

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
	if (serialThread->connect(ui->lineEdit->text().toLatin1().data()))
	{
		ui->statusBar->showMessage("Successfully connected!",2000);
		ui->btnConnect->setEnabled(false);
		ui->btnDisconnect->setEnabled(true);
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
	if (lastUpdate.elapsed() < 100) return;

	gloveWidget->update_data(data);
	updateJointBar(data[14]);
	lastUpdate.restart();
}

void MainWindow::updateJointBar(unsigned short data)
{
	const int min=4095;
	const int max=2000;
	const int targetRange=100;
	emit jointChanged(((data-min) * targetRange) / (max-min));
}

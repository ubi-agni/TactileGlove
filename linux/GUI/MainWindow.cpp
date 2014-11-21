#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "GloveWidget.h"

MainWindow::MainWindow(QWidget *parent) :
   QMainWindow(parent),
   ui(new Ui::MainWindow)
{
	ui->setupUi(this);
	ui->verticalLayout->insertWidget(1, gsp = new GloveWidget);
	serialThread = new SerialThread;
	ui->pushButtonDisconnect->setEnabled(false);
	connect(serialThread, SIGNAL ( read_frame(unsigned short*) ),
	        gsp, SLOT ( update_data(unsigned short*) ));
	connect(serialThread, SIGNAL ( read_frame(unsigned short*) ),
	        this, SLOT ( updateJointBar (unsigned short*) ));
	connect (serialThread, SIGNAL ( full_frame_update_message (QString)),
	         ui->statusBar, SLOT (showMessage (QString)));
	serialThread->start();
}

MainWindow::~MainWindow()
{
	serialThread->disconnect_device();
	serialThread->endthread();
	serialThread->wait();
	delete ui;
}

void MainWindow::on_pushButtonConnect_clicked()
{

	ui->statusBar->showMessage (QString ("Connecting..."), 2000);
	if (serialThread->connect_device(ui->lineEdit->text().toLatin1().data()))
	{
		ui->statusBar->showMessage("Successfully connected!",2000);
		ui->pushButtonConnect->setEnabled(false);
		ui->pushButtonDisconnect->setEnabled(true);
	}
	else
	{
		ui->statusBar->showMessage("Connection failed!",2000);
	}
}

void MainWindow::on_pushButtonDisconnect_clicked()
{
	ui->statusBar->showMessage("Disconnecting...",2000);
	ui->pushButtonDisconnect->setEnabled(false);
	serialThread->disconnect_device();
	gsp->reset_data();
	emit gsp->repaint();
	ui->statusBar->showMessage("Disconnected!",2000);
	ui->pushButtonConnect->setEnabled(true);
}

void MainWindow::updateJointBar(unsigned short *data)
{
	const int min=4095;
	const int max=2000;
	const int targetRange=ui->jointBar->maximum();
	const int val=data[14];
	ui->jointBar->setValue(((val-min) * targetRange) / (max-min));
}

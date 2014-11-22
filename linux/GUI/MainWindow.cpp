#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "GloveWidget.h"

GloveVizMainWindow::GloveVizMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GloveVizMainWindow)
{
    ui->setupUi(this);
    ui->verticalLayout->insertWidget(1, gsp = new GloveSvgPainter);
    seriallineconnector = new SerialLineConnector;
    ui->pushButtonDisconnect->setEnabled(false);
    connect(seriallineconnector, SIGNAL ( read_frame(unsigned short*) ),
            gsp, SLOT ( new_glove_data_available(unsigned short*) ));
    connect(seriallineconnector, SIGNAL ( read_frame(unsigned short*) ),
            this, SLOT ( updateJointBar (unsigned short*) ));
    connect (seriallineconnector, SIGNAL ( full_frame_update_message (QString)),
             ui->statusBar, SLOT (showMessage (QString)));
    connect (gsp, SIGNAL (ready_for_more() ),
             seriallineconnector, SLOT (enable_send ()));
    seriallineconnector->start();
}

GloveVizMainWindow::~GloveVizMainWindow()
{
    seriallineconnector->disconnect_device();
    seriallineconnector->endthread();
    seriallineconnector->wait();
    delete ui;
}

void GloveVizMainWindow::on_pushButtonConnect_clicked()
{

    ui->statusBar->showMessage (QString ("Connecting..."), 2000);
    if (seriallineconnector->connect_device(ui->lineEdit->text().toLatin1().data()))
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

void GloveVizMainWindow::on_pushButtonDisconnect_clicked()
{
    ui->statusBar->showMessage("Disconnecting...",2000);
    ui->pushButtonDisconnect->setEnabled(false);
    seriallineconnector->disconnect_device();
    gsp->reset_glove_data();
    emit gsp->repaint();
    ui->statusBar->showMessage("Disconnected!",2000);
    ui->pushButtonConnect->setEnabled(true);
}

void GloveVizMainWindow::updateJointBar(unsigned short *data)
{
    const int min=4095;
    const int max=2000;
    const int targetRange=ui->jointBar->maximum();
    const int val=data[14];
    ui->jointBar->setValue(((val-min) * targetRange) / (max-min));
}

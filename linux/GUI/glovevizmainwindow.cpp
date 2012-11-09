#include "glovevizmainwindow.h"



GloveVizMainWindow::GloveVizMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GloveVizMainWindow)
{
    ui->setupUi(this);
    ui->kled->off();
    vbl = new QVBoxLayout (ui->tab);
    gsp = new GloveSvgPainter;
    seriallineconnector = new SerialLineConnector;
    vbl->addWidget(gsp);
    gsp->show();
    ui->pushButtonDisconnect->setEnabled(false);
    QObject::connect((QObject*)seriallineconnector, SIGNAL ( read_frame(unsigned short*) ),
                     (QObject*)gsp, SLOT ( new_glove_data_available(unsigned short*) ));
    QObject::connect ((QObject*)seriallineconnector, SIGNAL ( full_frame_update_message (QString)),
                      (QObject*)ui->statusBar, SLOT (showMessage (QString)));
    seriallineconnector->start();
}

GloveVizMainWindow::~GloveVizMainWindow()
{
    seriallineconnector->disconnect_device();
    seriallineconnector->endthread();
    seriallineconnector->wait();
    delete ui;
}

void GloveVizMainWindow::paintEvent(QPaintEvent *)
 {

 }

void GloveVizMainWindow::on_pushButtonConnect_clicked()
{

    ui->statusBar->showMessage (QString ("Connecting..."), 2000);
    if (seriallineconnector->connect_device(ui->lineEdit->text().toAscii().data()))
    {
        ui->kled->on();
        ui->statusBar->showMessage("Successfully connected!",2000);
        ui->pushButtonConnect->setEnabled(false);
        ui->pushButtonDisconnect->setEnabled(true);
    }
    else
    {
        ui->kled->off();
        ui->statusBar->showMessage("Connection failed!",2000);
    }
}

void GloveVizMainWindow::on_pushButtonDisconnect_clicked()
{
    ui->statusBar->showMessage("Disconnecting...",2000);
    ui->pushButtonDisconnect->setEnabled(false);
    seriallineconnector->disconnect_device();
    ui->kled->off();
    gsp->reset_glove_data();
    emit gsp->repaint();
    ui->statusBar->showMessage("Disconnected!",2000);
    ui->pushButtonConnect->setEnabled(true);
}
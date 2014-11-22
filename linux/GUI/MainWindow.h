#pragma once

#include <QMainWindow>
#include "SerialThread.h"

namespace Ui {
    class GloveVizMainWindow;
}
class GloveSvgPainter;
class SerialLineConnector;

class GloveVizMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit GloveVizMainWindow(QWidget *parent = 0);
    ~GloveVizMainWindow();

private slots:
    void on_pushButtonConnect_clicked();
    void on_pushButtonDisconnect_clicked();
    void updateJointBar(unsigned short*);

private:
    Ui::GloveVizMainWindow *ui;
    GloveSvgPainter *gsp;
    SerialLineConnector* seriallineconnector;
};

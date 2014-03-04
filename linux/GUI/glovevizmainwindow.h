#ifndef GLOVEVIZMAINWINDOW_H
#define GLOVEVIZMAINWINDOW_H

#include <QMainWindow>
#include <QPainter>
#include <QtSvg>
#include "glovesvgpainter.h"
#include "seriallineconnector.h"
#include "ui_glovevizmainwindow.h"

namespace Ui {
    class GloveVizMainWindow;
}

class GloveVizMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit GloveVizMainWindow(QWidget *parent = 0);
    ~GloveVizMainWindow();

private slots:
    void on_pushButtonConnect_clicked();
    void on_pushButtonDisconnect_clicked();

private:
    Ui::GloveVizMainWindow *ui;
    GloveSvgPainter *gsp;
    SerialLineConnector* seriallineconnector;
};

#endif // GLOVEVIZMAINWINDOW_H

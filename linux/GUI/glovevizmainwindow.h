#ifndef GLOVEVIZMAINWINDOW_H
#define GLOVEVIZMAINWINDOW_H

#include <QMainWindow>
#include <QPainter>
#include <QtSvg>
#include "svgwindow.h"
#include "glovesvgpainter.h"
#include "seriallineconnector.h"
#include "ui_glovevizmainwindow.h"
#include "svgglview.h"

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
    QAction *nativeAction;
    QAction *glAction;
    QAction *imageAction;
    QAction *highQualityAntialiasingAction;
    SvgWindow *area;
    GloveSvgPainter *gsp;
    QVBoxLayout* vbl;
    SerialLineConnector* seriallineconnector;
};

#endif // GLOVEVIZMAINWINDOW_H

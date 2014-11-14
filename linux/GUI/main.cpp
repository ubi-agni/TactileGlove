#include <QApplication>
#include "glovevizmainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    GloveVizMainWindow w;
    w.show();

    return a.exec();
}

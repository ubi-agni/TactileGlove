#include <QtGui>

#include "svgglview.h"
#include "svgwindow.h"

SvgWindow::SvgWindow()
     : QScrollArea(),
       highQualityAntialiasing(false)
 {
     view = new QWidget(this);
     renderer = SvgWindow::OpenGL;
     setWidget(view);
 }

 void SvgWindow::openFile(const QString &file)
 {
     currentPath = file;
     setRenderer(renderer);
 }

 void SvgWindow::setRenderer(RendererType type)
 {
     renderer = type;

     if (renderer == OpenGL) {
         #ifndef QT_NO_OPENGL
         view = new SvgGLView(currentPath, this);
         qobject_cast<SvgGLView *>(view)->setHighQualityAntialiasing(highQualityAntialiasing);
         #endif
     } else if (renderer == Image) {
         view = new SvgRasterView(currentPath, this);
     } else {
         view = new SvgNativeView(currentPath, this);
     }

     setWidget(view);
     view->show();
 }

 void SvgWindow::setHighQualityAntialiasing(bool hq)
 {
     highQualityAntialiasing = hq;

     #ifndef QT_NO_OPENGL
     view = widget();
     if (renderer == OpenGL)
         qobject_cast<SvgGLView *>(view)->setHighQualityAntialiasing(highQualityAntialiasing);
     #endif
 }

 void SvgWindow::mousePressEvent(QMouseEvent *event)
 {
     mousePressPos = event->pos();
     scrollBarValuesOnMousePress.rx() = horizontalScrollBar()->value();
     scrollBarValuesOnMousePress.ry() = verticalScrollBar()->value();
     event->accept();
 }

 void SvgWindow::mouseMoveEvent(QMouseEvent *event)
 {
     if (mousePressPos.isNull()) {
         event->ignore();
         return;
     }

     horizontalScrollBar()->setValue(scrollBarValuesOnMousePress.x() - event->pos().x() + mousePressPos.x());
     verticalScrollBar()->setValue(scrollBarValuesOnMousePress.y() - event->pos().y() + mousePressPos.y());
     horizontalScrollBar()->update();
     verticalScrollBar()->update();
     event->accept();
 }

 void SvgWindow::mouseReleaseEvent(QMouseEvent *event)
 {
     mousePressPos = QPoint();
     event->accept();
 }


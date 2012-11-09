#ifndef SVGGLVIEW_H
#define SVGGLVIEW_H

#include <QWidget>
#include <QImage>
#include <QtOpenGL/QGLWidget>
#include <QtSvg>

class QPaintEvent;
class QSvgRenderer;
class QWheelEvent;

class SvgRasterView : public QWidget
 {
     Q_OBJECT

 public:
     SvgRasterView(const QString &file, QWidget *parent=0);

     virtual QSize sizeHint() const;


 protected slots:
     void poluteImage();
 protected:
     virtual void paintEvent(QPaintEvent *event);
     virtual void wheelEvent(QWheelEvent *event);

 private:
     QSvgRenderer *doc;
     QImage buffer;
     bool m_dirty;
 };

 class SvgNativeView : public QWidget
 {
     Q_OBJECT

 public:
     SvgNativeView(const QString &file, QWidget *parent=0);

     virtual QSize sizeHint() const;
 protected:
     virtual void paintEvent(QPaintEvent *event);
     virtual void wheelEvent(QWheelEvent *event);

 private:
     QSvgRenderer *doc;
 };

 #ifndef QT_NO_OPENGL
 class SvgGLView : public QGLWidget
 {
     Q_OBJECT

 public:
     SvgGLView(const QString &file, QWidget *parent=0);

     void setHighQualityAntialiasing(bool highQualityAntialiasing);
     void setElementColor (int i);
     virtual QSize sizeHint() const;
 protected:
     virtual void paintEvent(QPaintEvent *event);
     virtual void wheelEvent(QWheelEvent *event);

 private:
     QSvgRenderer *doc;

     bool highQualityAntialiasing;
 };
 #endif

#endif // SVGGLVIEW_H

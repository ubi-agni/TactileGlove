#ifndef GLOVESVGPAINTER_H
#define GLOVESVGPAINTER_H

#include <QWidget>
#include <QtSvg>
#include <QPainter>
#include <QDomDocument>
#include <iostream>


#define NO_GLOVE_ELEMENTS 62


typedef struct GloveData_T
{
  pthread_mutex_t* data_mutex;
  unsigned short data_array[NO_GLOVE_ELEMENTS];
} glovedata_t;


class GloveSvgPainter : public QWidget
{
    Q_OBJECT
public:
    explicit GloveSvgPainter(QWidget *parent = 0);
    void generate_random_glovedata ();
    void reset_glove_data();
protected:
    virtual void paintEvent(QPaintEvent *event);
signals:
    void ready_for_more ();
public slots:
    void new_glove_data_available(unsigned short* glove_update);
private:
    QDomDocument* qDomDocPtr;
    QSvgRenderer* qSvgRendererPtr;
    QDomNode qDomNodeArray[NO_GLOVE_ELEMENTS];
    glovedata_t* gd;
    void update_svg ();
    void init_glovedata();

};

#endif // GLOVESVGPAINTER_H

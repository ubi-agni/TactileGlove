#ifndef GLOVESVGPAINTER_H
#define GLOVESVGPAINTER_H

#include <QWidget>
#include <QtSvg>
#include <QPainter>
#include <iostream>
extern "C" {
#include "svgint.h"
#include "svg-cairo-internal.h"
#include <cairo/cairo-svg.h>
#include <pthread.h>
#include <cairo/cairo-features.h>
#include <cairo/cairo.h>
#include <cairo/cairo-qt.h>
}

#define NO_GLOVE_ELEMENTS 54




static const char lookup[NO_GLOVE_ELEMENTS][9] =
    {"path3449","path3451","path3411","path3447","path3415","path3413",
     "path3467","path3465","path3469","path3471","path3359","path3461",
     "path3459","path3361","path3363",
     "path3365","path3369","path3367","path3371","path3381","path3377","path3375","path3357",
     "path3475",
     "path3481","path3479","path3477","path3383","path3385","path3387",
     "path3457","path3389","path3391","path3395","path3393","path3397",
     "path3401","path3403","path3409","path3407",
     "path3419","path3421","path3423","path3425",
     "path3431","path3429","path3427","path3433",
     "path3437","path3439","path3435","path3443","path3441",
     "path3445"};

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

public slots:
    void new_glove_data_available(unsigned short* glove_update);
private:
    glovedata_t* gd;
    void update_svg ();
    svg_cairo_t* scr;
    void init_glovedata();

};

#endif // GLOVESVGPAINTER_H

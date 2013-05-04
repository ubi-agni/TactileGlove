#ifndef GLOVESVGPAINTER_H
#define GLOVESVGPAINTER_H

#include <QWidget>
#include <QtSvg>
#include <QPainter>
#include <QDomDocument>
#include <iostream>


#define NO_GLOVE_ELEMENTS 54




static const char lookup[NO_GLOVE_ELEMENTS][9] =
//	A(tip)	    B(left)	C(middle)  D(right)  E		F	   G	      H		 I
    {"path3449","path3451","path3411","path3447","path3415","path3413",//1 Thumb
     "path3467","path3465","path3469","path3471","path3359","path3461","path3459","path3361","path3363",//2 Pointer
     "path3365","path3369","path3367","path3371","path3381","path3377","path3375","path3357",		//3 Middle finger
     "path3475","path3481","path3479","path3477","path3383","path3385","path3387","path3457",		//4 Ring finger
     "path3389","path3391","path3395","path3393","path3397","path3401","path3403","path3409","path3407",//5 Little finger
     "path3419","path3421","path3423","path3425",		//6 Heel of the Hand (line under the fingers) pad
     "path3431","path3429","path3433","path3427",		//7 Ball of the thumb pad
     "path3437","path3439","path3435","path3443","path3441",	//8 Ball of the Hand (middle) pad
     "path3445"};						//9 Side of the hand pad

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
    QDomDocument* qDomDocPtr;
    QSvgRenderer* qSvgRendererPtr;
    glovedata_t* gd;
    void update_svg ();
    void init_glovedata();

};

#endif // GLOVESVGPAINTER_H

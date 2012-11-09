#include "glovesvgpainter.h"

void
GloveSvgPainter::init_glovedata ()
{
  int i;
  gd->data_mutex = (pthread_mutex_t*) malloc (sizeof (pthread_mutex_t));
  if (gd->data_mutex == NULL)
    {
      perror ("init_glovedata: malloc");
      exit (EXIT_FAILURE);
    }

  pthread_mutex_init (gd->data_mutex, NULL);
  for (i = 0; i < NO_GLOVE_ELEMENTS; i++)
    gd->data_array[i] = 0;

}

void
GloveSvgPainter::reset_glove_data()
{
    if (0 != pthread_mutex_lock(gd->data_mutex))
    {
        perror ("GloveSvgPainter::reset_glove_data():pthreat_mutex_lock");
        exit (EXIT_FAILURE);
    }
    for (int i = 0; i < NO_GLOVE_ELEMENTS; i++)
      gd->data_array[i] = 0;
    if (0 != pthread_mutex_unlock(gd->data_mutex))
    {
        perror ("GloveSvgPainter::reset_glove_data():pthreat_mutex_unlock");
        exit (EXIT_FAILURE);
    }
}

void
GloveSvgPainter::new_glove_data_available(unsigned short* glove_update)
{
    int i;

    if (0 != pthread_mutex_lock (gd->data_mutex))
    {
        perror ("GloveSvgPainter::new_glove_data_available: pthread_mutex_lock");
        exit (EXIT_FAILURE);
    }
    for (i=0; i < NO_GLOVE_ELEMENTS; i++)
    {
        gd->data_array[i] = glove_update[i];
    }
    if (0 != pthread_mutex_unlock (gd->data_mutex))
    {
        perror ("GloveSvgPainter::new_glove_data_available: pthread_mutex_unlock");
        exit (EXIT_FAILURE);
    }
    repaint();
}

void
GloveSvgPainter::generate_random_glovedata ()
{
  int i;

  long int rndnumber;

  if (0 != pthread_mutex_lock (gd->data_mutex))
    {
      perror ("generate_new_glovedata: pthread_mutex_lock");
      exit (EXIT_FAILURE);
    }

  for (i=0; i < NO_GLOVE_ELEMENTS; i++)
    {
      rndnumber = random();

      if (rndnumber < (RAND_MAX / 2)) /* only give new value 50% of time */
        gd->data_array[i] = rndnumber&0xFFF;
    }
  if (0 != pthread_mutex_unlock (gd->data_mutex))
    {
      perror ("generate_new_glovedata: pthread_mutex_unlock");
      exit (EXIT_FAILURE);
    }
}
GloveSvgPainter::GloveSvgPainter(QWidget *parent) :
    QWidget(parent)
{
    gd = (glovedata_t*) malloc (sizeof (glovedata_t));
    if (NULL == gd)
    {
        perror ("main(): malloc");
        exit (EXIT_FAILURE);
    }
    svg_cairo_create(&scr);
    svg_cairo_parse (scr, "Sensorlayout04.svg");
    init_glovedata ();
}

void GloveSvgPainter::paintEvent(QPaintEvent *event)
{
    cairo_surface_t* cairo_surface;
    cairo_t* cr;
    QPainter painter(this);
    painter.setRenderHint(QPainter::HighQualityAntialiasing,true);
    update_svg();
    cairo_surface = cairo_qt_surface_create (&painter);
    cr = cairo_create(cairo_surface);
    svg_cairo_render (scr, cr);
    cairo_destroy (cr);
    painter.restore();
}

void
GloveSvgPainter::update_svg()
{
    unsigned int height, width;
    int i;
    svg_element_t* se;
    svg_cairo_get_size (scr, &width, &height);

    if (0 != pthread_mutex_lock(gd->data_mutex))
    {
        perror ("GloveSvgPainter::update_svg: pthread_mutex_lock");
        exit (EXIT_FAILURE);
    }
   // fprintf (stderr," DEBUG: Frame data: ");
    for (i=0; i < NO_GLOVE_ELEMENTS; i++)
    {
      unsigned int temp = gd->data_array[i];
        se = _svg_fetch_element_by_id (scr->svg,lookup[i]);
	//temp = 4*temp; //anpassen der werte mit einer Funktion
	//if (temp > 4095) temp = 4095;
	//if (temp <= 1365)
	  {
	    se->node->style.fill_paint.p.color.rgb = 0x100*(((1000*temp / 5353) > 255)?255:(1000*temp / 5353));
	  }
	else
	  {
	    if (temp <= 2730)
	      {
		se->node->style.fill_paint.p.color.rgb = (1000*(temp-1365) / 5353)*0x10000 + 0xff00;
	      }
	    else
	      {
		se->node->style.fill_paint.p.color.rgb = 0x100*(0xff - (1000*(temp-2730) / 5353)) + 0xff0000;
	      }
	  }
	//fprintf (stderr,"%d ",temp);
    }
    //fprintf (stderr,"\n");
    if (0 != pthread_mutex_unlock (gd->data_mutex))
    {
        perror ("GloveSvgPainter::update_svg: pthread_mutex_unlock");
        exit (EXIT_FAILURE);
    }

    //cairo_scale (cr,(double) screen->w/width,(double)screen->h/height);

}

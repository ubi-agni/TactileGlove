#pragma once

#include <QWidget>
#include <QDomNode>
#include <QMutex>

#define NO_TAXELS 64

class QDomDocument;
class QDomNode;
class QSvgRenderer;
class QAction;

class GloveWidget : public QWidget
{
	Q_OBJECT
public:
	explicit GloveWidget(QWidget *parent = 0);
	QSize sizeHint() const;

public slots:
	void update_data(unsigned short* data);
	void reset_data();

private:
	void paintEvent(QPaintEvent *event);
	void update_svg();

private:
	QDomDocument  *qDomDocPtr;
	QSvgRenderer  *qSvgRendererPtr;
	QDomNode       qDomNodeArray[NO_TAXELS];
	QAction       *actShowChannels;
	QAction       *actShowIDs;
	unsigned short data[NO_TAXELS];
};


#pragma once

#include "TaxelMapping.h"
#include <QWidget>
#include <QDomNode>
#include <QMutex>
#include <QVector>

#define NO_TAXELS 64

class QDomDocument;
class QDomNode;
class QSvgRenderer;
class QAction;

class GloveWidget : public QWidget
{
	Q_OBJECT
public:
	explicit GloveWidget(const QString& sLayout,
	                     const TaxelMapping& mapping,
	                     QWidget *parent = 0);
	QSize sizeHint() const;

public slots:
	void update_data(unsigned short* data);
	void reset_data();

private:
	void paintEvent(QPaintEvent *event);
	void update_svg();
	bool event(QEvent *event);
	int  channelAt(const QPoint &p);

private:
	QDomDocument  *qDomDocPtr;
	QSvgRenderer  *qSvgRendererPtr;
	QDomNode       qDomNodeArray[NO_TAXELS];
	QAction       *actShowChannels;
	QAction       *actShowIDs;
	QTransform     viewTransform;
	unsigned short data[NO_TAXELS];
	QVector<QString> pathNames;
};


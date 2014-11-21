#pragma once

#include <QWidget>
#include <QDomNode>

#define NO_TAXELS 64

class QDomDocument;
class QDomNode;
class QSvgRenderer;

class GloveWidget : public QWidget
{
	Q_OBJECT
public:
	explicit GloveWidget(QWidget *parent = 0);
	QSize sizeHint() const;
	int heightForWidth(int) const;

public slots:
	void update_data(unsigned short* data);
	void reset_data();

protected:
	virtual void paintEvent(QPaintEvent *event);
	void update_svg();

private:
	QDomDocument  *qDomDocPtr;
	QSvgRenderer  *qSvgRendererPtr;
	QDomNode       qDomNodeArray[NO_TAXELS];
	unsigned short data[NO_TAXELS];
	bool           bDirty;
};


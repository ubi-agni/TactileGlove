#include "GloveWidget.h"

#include <QtSvg>
#include <QDomDocument>
#include <stdexcept>
#include <assert.h>
#include <iostream>

using namespace std;

static const QStringList& pathNames() {
	static QStringList names;
	if (names.isEmpty()) {
		names << "" << "" << "" << "" << "" << "LF12" << "LF11" << "LF13" << "LF23" << "LF14" // 1..10
		      << "LF21" << "LF22" << "LF32" << "LF31" << "" << "" << "FF12" << "FF11" << "FF14" << "FF23" // 11..20
		      << "FF21" << "TH13" << "TH11" << "TH12" << "FF31" << "TH14" << "TH22" << "TH21" << "FF22" << "FF13" // 21..30
		      << "FF32" << "RF23" << "RF13" << "RF12" << "RF11" << "RF22" << "RF21" << "MF23" << "MF21" << "MF11" // 31..40
		      << "MF12" << "MF31" << "MF14" << "MF22" << "MF13" << "" << "RF14" << "RF31" << "PM34" << "PM32" // 41..50
		      << "PM31" << "PM21" << "" << "PM14" << "PM11" << "PM13" << "PM44" << "PM41" << "PM43" << "PM12" // 51..60
		      << "PM42" << "PM33" << "PM35" << ""; // 61..64
		assert(names.size() == NO_TAXELS);
		assert(QSet<QString>::fromList(names).size() == 54+1);
	}
	return names;
}

GloveWidget::GloveWidget(QWidget *parent) : QWidget(parent)
{
	qDomDocPtr = new QDomDocument ("Sensorlayout");
	QFile file(":Sensorlayout04.svg");
	if (!file.open(QIODevice::ReadOnly))
		throw std::runtime_error("failed to open sensor layout");

	QString errorMsg;
	if (!qDomDocPtr->setContent(&file, &errorMsg)) {
		file.close();
		throw std::runtime_error(errorMsg.toStdString());
	}
	file.close();

	qSvgRendererPtr = new QSvgRenderer (this);
	QDomElement docElem = qDomDocPtr->documentElement();
	QDomNode gRoot, gLevel1, pathlevel;

	/* Finding nodes for the elements */
	if ((gRoot = docElem.firstChildElement(QString ("g"))).isNull())
		throw std::runtime_error("Invalid SVG document: no root node 'g'");

	if ((gLevel1 = gRoot.firstChildElement(QString("g"))).isNull())
		throw std::runtime_error("Invalid SVG document: no level-1 node 'g'");

	if ((pathlevel = gLevel1.firstChildElement(QString("path"))).isNull())
		throw std::runtime_error("Invalid SVG document: no path elements");

	for (; !pathlevel.isNull(); pathlevel = pathlevel.nextSiblingElement("path")) {
		QDomNamedNodeMap qmap = pathlevel.attributes();
		QDomNode nid = qmap.namedItem(QString("id"));
		if (nid.isNull()) continue;

		int i = pathNames().indexOf(nid.nodeValue());
		if (i < 0) {
			if (nid.nodeValue().length() == 4)
				cerr << "didn't find node " << nid.nodeValue().toStdString();
			continue;
		}
		qDomNodeArray[i] = qmap.namedItem(QString("style"));
	}
	reset_data();

	QSizePolicy sp(QSizePolicy::Preferred, QSizePolicy::Preferred);
	sp.setHeightForWidth(true);
	setSizePolicy(sp);
}

QSize GloveWidget::sizeHint() const
{
	return qSvgRendererPtr->defaultSize();
}

int GloveWidget::heightForWidth(int w) const
{
	const QSize &s = qSvgRendererPtr->defaultSize();
	return w*s.height()/s.width();
}

void GloveWidget::paintEvent(QPaintEvent * /*event*/)
{
	QPainter painter(this);
	painter.setRenderHint(QPainter::HighQualityAntialiasing,false);
	qSvgRendererPtr->render(&painter);
}

void GloveWidget::update_data(unsigned short *data)
{
	bool bDirty = false;
	for (int i=0; i < NO_TAXELS; ++i)
		if (this->data[i] != data[i]) {
			bDirty = true;
			break;
		}
	if (!bDirty) return;

	std::copy(data, data + NO_TAXELS, this->data);
	update_svg();
}

void GloveWidget::reset_data()
{
	bzero(data, sizeof(data));
	update_svg();
}

void GloveWidget::update_svg()
{
	for (int i=0; i < NO_TAXELS; ++i) {
		char value[256];
		unsigned int temp = data[i];
		unsigned int color;

		if (temp > 4095) temp = 4095;
		if (temp <= 1365)
			color = 0x100*(((1000*temp / 5353) > 255)?255:(1000*temp / 5353));
		else {
			if (temp <= 2730)
				color = (1000*(temp-1365) / 5353)*0x10000 + 0xff00;
			else
				color = 0x100*(0xff - (1000*(temp-2730) / 5353)) + 0xff0000;
		}
		snprintf (value,256,"fill:#%06x;stroke=none", color);
		qDomNodeArray[i].setNodeValue(QString(value));
	}
	qSvgRendererPtr->load(qDomDocPtr->toByteArray());
	update();
}

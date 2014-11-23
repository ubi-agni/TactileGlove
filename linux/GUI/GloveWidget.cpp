#include "GloveWidget.h"

#include <QtSvg>
#include <QDomDocument>
#include <stdexcept>

static const QStringList& pathNames() {
	static QStringList names;
	if (names.isEmpty()) {
		names << "" << "path3443" << "path3419" << "path3427" << "path3431" << "path3393" //1 Thumb
		      << "path3389" << "path3395" << "path3397" << "path3391" << "path3403" << "path3401" << "path3409" << "path3407" << "path3433" //2 Index finger
		      << "path3423" << "path3471" << "path3467" << "path3465" << "path3359" << "path3459" << "path3411" << "path3449" //3 Middle finger
		      << "path3447" << "path3363" << "path3451" << "path3415" << "path3413" << "path3461" << "path3469" << "path3361" //4 Ring finger
		      << "path3383" << "path3479" << "path3477" << "path3475" << "path3385" << "path3387" << "path3381" << "path3375" << "path3365" //5 Little finger
		      << "path3371" << "path3357" << "path3369" << "path3377" //6 Heel of the Hand (line under the fingers) pad
		      << "path3367" << "path3429" << "path3481" << "path3457" //7 Ball of the thumb pad
		      << "path3439" << "path3435" << "path3441" << "path3445" << "path3437" //8 Ball of the Hand (middle) pad
		      << "path3419" << "path3425" << "path3421" << "path3431" << "path3433" << "path3429" << "path3423" << "path3427" << "path3437"; //9 Side of the hand pad
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
		if (i < 0) continue;
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

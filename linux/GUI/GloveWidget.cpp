#include "GloveWidget.h"

#include <QtSvg>
#include <QDomDocument>
#include <QAction>
#include <stdexcept>
#include <assert.h>
#include <iostream>
#include <boost/format.hpp>

using namespace std;

static QString getLabel(int channel, const QString &id, bool showChannel=true, bool showID=true) {
	QString result;
	if (showChannel) result.setNum(channel);
	if (showChannel && showID) result.append(": ");
	if (showID) result.append(id);
	return result;
}

GloveWidget::GloveWidget(const QString &sLayout, const TaxelMapping &mapping, QWidget *parent)
   : QWidget(parent)
{
	qDomDocPtr = new QDomDocument ("Sensorlayout");
	QFile file(QString(":%1.svg").arg(sLayout));
	if (!file.open(QIODevice::ReadOnly))
		throw std::runtime_error("failed to open sensor layout");

	QString errorMsg;
	if (!qDomDocPtr->setContent(&file, &errorMsg)) {
		file.close();
		throw std::runtime_error(errorMsg.toStdString());
	}
	file.close();

	actShowChannels = new QAction("show channels", this);
	actShowChannels->setCheckable(true);
	actShowChannels->setChecked(false);
	connect(actShowChannels, SIGNAL(toggled(bool)), this, SLOT(update()));
	actShowIDs = new QAction("show SVG ids", this);
	actShowIDs->setCheckable(true);
	actShowIDs->setChecked(false);
	connect(actShowIDs, SIGNAL(toggled(bool)), this, SLOT(update()));

	this->addAction(actShowChannels);
	this->addAction(actShowIDs);
	this->setContextMenuPolicy(Qt::ActionsContextMenu);

	// create pathNames StringList from TaxelMapping
	pathNames.resize(NO_TAXELS);
	for (TaxelMapping::const_iterator it=mapping.begin(), end=mapping.end();
	     it!=end; ++it) {
		if (it->second >= NO_TAXELS)
			throw std::runtime_error(boost::str(boost::format("invalid taxel channel %s=%d")
		                                       % it->first % it->second));
		pathNames[it->second] = QString::fromStdString(it->first);
	}
	qSvgRendererPtr = new QSvgRenderer (this);
	QDomElement docElem = qDomDocPtr->documentElement();

	/* Finding path elements */
	QDomNodeList paths = docElem.elementsByTagName("path");
	for (int i=0; i<paths.count(); ++i) {
		QDomNode path = paths.at(i);
		QDomNamedNodeMap qmap = path.attributes();
		QDomNode nid = qmap.namedItem(QString("id"));
		if (nid.isNull()) continue;

		int idx = pathNames.indexOf(nid.nodeValue());
		if (idx < 0) continue;
		qDomNodeArray[idx] = qmap.namedItem(QString("style"));
	}
	/* check whether we found all path elements */
	for (int i=0; i<pathNames.count(); ++i) {
		if (!pathNames.at(i).isEmpty() && qDomNodeArray[i].isNull())
			cerr << "couldn't find a node named " << pathNames.at(i).toStdString() << endl;
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

void GloveWidget::paintEvent(QPaintEvent * /*event*/)
{
	QPainter painter(this);
	painter.setRenderHint(QPainter::HighQualityAntialiasing,false);

	// setup correct scaling
	const QSize& svgSize = qSvgRendererPtr->defaultSize();
	QSize renderSize(svgSize);
	renderSize.scale(painter.viewport().size(), Qt::KeepAspectRatio);
	painter.setWindow(0,0, svgSize.width(), svgSize.height()); // logical coordinates are fixed
	painter.setViewport(0,0, renderSize.width(), renderSize.height());
	viewTransform = painter.combinedTransform();

	qSvgRendererPtr->render(&painter, painter.window());

	if (!actShowIDs->isChecked() &&
	    !actShowChannels->isChecked())
		return;

	// show IDs/channels of taxels
	QFont font = painter.font(); font.setPointSize(8);
	painter.setFont(font);

	int channel=1;
	for (QVector<QString>::const_iterator it=pathNames.begin(), end=pathNames.end();
	     it != end; ++it, ++channel) {
		if (it->isEmpty()) continue;
		QMatrix m = qSvgRendererPtr->matrixForElement(*it);
		QRectF bounds = m.mapRect(qSvgRendererPtr->boundsOnElement(*it));
		QString label = getLabel(channel, *it,
		                         actShowChannels->isChecked(),
		                         actShowIDs->isChecked());
		painter.setPen(Qt::black);
		painter.drawText(bounds.translated(1,1), Qt::AlignCenter, label);
		painter.setPen(Qt::white);
		painter.drawText(bounds, Qt::AlignCenter, label);
	}
}

int  GloveWidget::channelAt(const QPoint &p) {
	int channel=1;
	for (QVector<QString>::const_iterator it=pathNames.begin(), end=pathNames.end();
	     it != end; ++it, ++channel) {
		if (it->isEmpty()) continue;
		QMatrix m = qSvgRendererPtr->matrixForElement(*it);
		QRectF bounds = m.mapRect(qSvgRendererPtr->boundsOnElement(*it));
		if (bounds.contains(p)) return channel;
	}
	return -1;
}

bool GloveWidget::event(QEvent *event)
{
	if (event->type() == QEvent::ToolTip) {
		QHelpEvent *helpEvent = static_cast<QHelpEvent *>(event);
		int ch = channelAt(viewTransform.inverted().map(helpEvent->pos()));
		if (ch != -1) {
			QToolTip::showText(helpEvent->globalPos(), getLabel(ch, pathNames.at(ch-1)));
		} else {
			QToolTip::hideText();
			event->ignore();
		}
		return true;
	}
	return QWidget::event(event);
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

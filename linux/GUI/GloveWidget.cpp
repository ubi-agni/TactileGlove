#include "GloveWidget.h"

#include <QtSvg>
#include <QDomDocument>
#include <QAction>
#include <stdexcept>
#include <assert.h>
#include <iostream>

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
	QFile file(sLayout);
	if (!file.exists()) file.setFileName(QString(":%1.svg").arg(sLayout));
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
	actShowAllIDs = new QAction("show all SVG ids", this);
	actShowAllIDs->setCheckable(true);
	actShowAllIDs->setChecked(false);
	connect(actShowAllIDs, SIGNAL(toggled(bool)), this, SLOT(update()));
	// actShowIDs+actShowChannels are mutually exclusive with actShowAllIDs:
	connect(actShowIDs, SIGNAL(toggled(bool)), actShowAllIDs, SLOT(setDisabled(bool)));
	connect(actShowAllIDs, SIGNAL(toggled(bool)), actShowIDs, SLOT(setDisabled(bool)));
	connect(actShowChannels, SIGNAL(toggled(bool)), actShowAllIDs, SLOT(setDisabled(bool)));
	connect(actShowAllIDs, SIGNAL(toggled(bool)), actShowChannels, SLOT(setDisabled(bool)));

	actSaveMapping = new QAction("save taxel mapping", this);
	connect(actSaveMapping, SIGNAL(triggered()), this, SLOT(save_mapping()));

	QAction* separator=new QAction(this); separator->setSeparator(true);
	this->addAction(actShowChannels);
	this->addAction(actShowIDs);
	this->addAction(actShowAllIDs);
	this->addAction(separator);
	this->addAction(actSaveMapping);
	this->setContextMenuPolicy(Qt::ActionsContextMenu);

	// retrieve mapping from all node names to their style attribute items
	QDomNodeList paths = qDomDocPtr->documentElement().elementsByTagName("path");
	for (int i=0; i<paths.count(); ++i) {
		QDomNode path = paths.at(i);
		QDomNamedNodeMap qmap = path.attributes();
		QString name = qmap.namedItem(QString("id")).nodeValue();
		QDomNode  sn = qmap.namedItem(QString("style"));
		if (name.isEmpty() || sn.isNull()) continue;
		allNodes[name] = sn;
	}

	// realize TaxelMapping
	for (TaxelMapping::const_iterator it=mapping.begin(), end=mapping.end();
	     it!=end; ++it) {
		QString sName = QString::fromStdString(it->first);
		if (!assign(sName, it->second))
			cerr << "couldn't find a node named " << it->first << endl;
	}

	qSvgRendererPtr = new QSvgRenderer (this);
	reset_data();

	QSizePolicy sp(QSizePolicy::Preferred, QSizePolicy::Preferred);
	sp.setHeightForWidth(true);
	setSizePolicy(sp);
}

const QDomNode& GloveWidget::find(const QString &sNode) const {
	static const QDomNode dummy;
	StyleNodeMap::const_iterator it = allNodes.find(sNode);
	if (it == allNodes.end()) return dummy; // not found
	else return *it;
}

GloveWidget::TaxelInfo::TaxelInfo(unsigned short idx, const QDomNode &node)
   : channel(idx), styleNode(node)
{
	static const QString fillKey="fill:#";

	styleString = styleNode.nodeValue();
	iFillStart = styleString.indexOf(fillKey);
	if (iFillStart == -1) {
		// fillKey missing for node: add ourselves
		iFillStart = styleString.length() + 1 + fillKey.length();
		styleString.append(";fill:#ffffff");
	} else iFillStart += fillKey.length();
}

bool GloveWidget::assign(const QString &sName, int idx) {
	const QDomNode &styleNode = find(sName);
	if (styleNode.isNull()) return false;

	if (idx < 0 || idx >= NO_TAXELS)
		throw std::runtime_error(QString("invalid taxel channel %1=%2")
	                            .arg(sName).arg(idx).toStdString());

	taxels.insert(sName, TaxelInfo(idx, styleNode));
	return true;
}

void GloveWidget::unassign(const QString &sName) {
	taxels.remove(sName);
}

void GloveWidget::save_mapping()
{
	QString sFileName = QFileDialog::getSaveFileName(0, "save taxel mapping");
	QFile file(sFileName);
	if (!file.open(QFile::WriteOnly | QFile::Truncate)) {
		QMessageBox::warning(this, "save taxel mapping",
		                     QString("Failed to open file for writing:\n%1").arg(sFileName));
		return;
	}

	QTextStream ts(&file);
	// write associated mappings
	unsigned int iWritten=0;
	for (TaxelMap::const_iterator it=taxels.begin(), end=taxels.end(); it!=end; ++it) {
		ts << it.key() << "=" << it->channel << endl;
		++iWritten;
	}
	if (iWritten > NO_TAXELS / 2) return;

	// write all node names
	ts << endl;
	for (StyleNodeMap::const_iterator it=allNodes.begin(), end=allNodes.end();
	     it != end; ++it) {
		if (it.key().isEmpty() || taxels.find(it.key()) != taxels.end()) continue;
		ts << it.key() << "=" << endl;
	}
}

void GloveWidget::edit_mapping(const QString& node, int channel)
{
	if (node.isEmpty()) return;
	bool ok;
	int newChannel = QInputDialog::getInt(this, "adapt taxel mapping", QString("Map %1 to channel:").arg(node),
	                                      channel+1, 0, NO_TAXELS, 1, &ok);
	if (!ok) return;
	if (newChannel == 0) unassign(node);
	else assign(node, newChannel-1);
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

	QFont font = painter.font(); font.setPointSize(8);
	painter.setFont(font);

	// show IDs/channels of taxels
	if (actShowIDs->isEnabled() && actShowChannels->isEnabled() &&
	    (actShowIDs->isChecked() || actShowChannels->isChecked())) {
		for (TaxelMap::const_iterator it=taxels.begin(), end=taxels.end();
				  it != end; ++it) {
			const QString &sName = it.key();
			QMatrix m = qSvgRendererPtr->matrixForElement(sName);
			QRectF bounds = m.mapRect(qSvgRendererPtr->boundsOnElement(sName));
			QString label = getLabel(it->channel+1, sName,
											 actShowChannels->isChecked(),
											 actShowIDs->isChecked());
			painter.setPen(Qt::black);
			painter.drawText(bounds.translated(1,1), Qt::AlignCenter, label);
			painter.setPen(Qt::white);
			painter.drawText(bounds, Qt::AlignCenter, label);
		}
	}

	if (actShowAllIDs->isEnabled() && actShowAllIDs->isChecked()) {
		for (StyleNodeMap::const_iterator it=allNodes.begin(), end=allNodes.end();
		     it != end; ++it) {
			const QString &sName = it.key();
			QMatrix m = qSvgRendererPtr->matrixForElement(sName);
			QRectF bounds = m.mapRect(qSvgRendererPtr->boundsOnElement(sName));
			QString label = getLabel(0, sName, false, true);
			painter.setPen(Qt::black);
			painter.drawText(bounds.translated(1,1), Qt::AlignCenter, label);
			painter.setPen(Qt::white);
			painter.drawText(bounds, Qt::AlignCenter, label);
		}
	}
}

std::pair<QString, int> GloveWidget::pathAt(const QPoint &p) {
	for (TaxelMap::const_iterator it=taxels.begin(), end=taxels.end();
	     it != end; ++it) {
		const QString &sName = it.key();
		QMatrix m = qSvgRendererPtr->matrixForElement(sName);
		QRectF bounds = m.mapRect(qSvgRendererPtr->boundsOnElement(sName));
		if (bounds.contains(p)) return make_pair(sName, it->channel);
	}
	for (StyleNodeMap::const_iterator it=allNodes.begin(), end=allNodes.end();
	     it != end; ++it) {
		const QString &sName = it.key();
		if (sName.isEmpty()) continue;
		QMatrix m = qSvgRendererPtr->matrixForElement(sName);
		QRectF bounds = m.mapRect(qSvgRendererPtr->boundsOnElement(sName));
		if (bounds.contains(p)) return make_pair(sName, -1);
	}
	return make_pair(QString(), -1);
}

bool GloveWidget::event(QEvent *event)
{
	switch(event->type()) {
	case QEvent::ToolTip: {
		QHelpEvent *helpEvent = static_cast<QHelpEvent *>(event);
		pair<QString, int> info = pathAt(viewTransform.inverted().map(helpEvent->pos()));
		if (!info.first.isEmpty()) {
			QToolTip::showText(helpEvent->globalPos(), getLabel(info.second+1, info.first,
			                                                    info.second >= 0, true));
		} else {
			QToolTip::hideText();
			event->ignore();
		}
		return true;
	}
	case QEvent::MouseButtonDblClick:
		QMouseEvent *mouseEvent = static_cast <QMouseEvent *>(event);
		if (mouseEvent->button() == Qt::LeftButton) {
			pair<QString, int> info = pathAt(viewTransform.inverted().map(mouseEvent->pos()));
			edit_mapping(info.first, info.second);
			return true;
		}
		break;
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
	static const QString fmt("%1");
	for (TaxelMap::iterator it=taxels.begin(), end=taxels.end(); it!=end; ++it) {
		unsigned int temp = data[it->channel];
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
		// replace color string in style string
		it->styleString.replace(it->iFillStart, 6, fmt.arg(color, 6, 16, QLatin1Char('0')));
		it->styleNode.setNodeValue(it->styleString);
	}
	qSvgRendererPtr->load(qDomDocPtr->toByteArray());
	update();
}

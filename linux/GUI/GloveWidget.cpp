#include "GloveWidget.h"
#include "MappingDialog.h"

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
	qDomDocPtr = new QDomDocument (sLayout);
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
	actSaveSVG = new QAction("save SVG", this);
	connect(actSaveSVG, SIGNAL(triggered()), this, SLOT(save_svg()));

	QAction* separator=new QAction(this); separator->setSeparator(true);
	this->addAction(actShowChannels);
	this->addAction(actShowIDs);
	this->addAction(actShowAllIDs);
	this->addAction(separator);
	this->addAction(actSaveMapping);
	this->addAction(actSaveSVG);
	this->setContextMenuPolicy(Qt::ActionsContextMenu);

	// retrieve mapping from all node names to their style attribute items
	QDomNodeList paths = qDomDocPtr->documentElement().elementsByTagName("path");
	for (int i=0; i<paths.count(); ++i) {
		QDomNode path = paths.at(i);
		QDomNamedNodeMap qmap = path.attributes();
		QString name = qmap.namedItem("id").nodeValue();
		QDomNode  sn = qmap.namedItem("style");
		if (name.isEmpty() || sn.isNull()) continue;
		allNodes.push_front(make_pair(name, path));
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

int GloveWidget::findPathNodeIndex(const QString &sName) const {
	int idx=0;
	for (PathList::const_iterator it=allNodes.begin(), end=allNodes.end();
	     it != end; ++it, ++idx)
		if (it->first == sName) return idx;
	return -1; // not found
}

QDomNode GloveWidget::findStyleNode(const QString &sName) const
{
	int idx = findPathNodeIndex(sName);
	if (idx < 0) return QDomNode();
	QDomNamedNodeMap qmap = allNodes[idx].second.attributes();
	return qmap.namedItem("style");
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
	const QDomNode &styleNode = findStyleNode(sName);
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

void GloveWidget::save_svg()
{
	QString sFileName = QFileDialog::getSaveFileName(0, "save sensor layout",
	                                                 qDomDocPtr->nodeValue(), "*.svg");
	QFileInfo info(sFileName);
	sFileName.remove(sFileName.size()-1 - info.completeSuffix().size(), sFileName.size());
	sFileName.append(".svg");

	QFile file(sFileName);
	if (!file.open(QFile::WriteOnly | QFile::Truncate)) {
		QMessageBox::warning(this, "save sensor layout",
		                     QString("Failed to open file for writing:\n%1").arg(sFileName));
		return;
	}

	QTextStream ts(&file);
	qDomDocPtr->save(ts, 2);
	qDomDocPtr->setNodeValue(sFileName);
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
	// nicely sort name=channel pairs by (channel, name)
	typedef std::set<std::pair<unsigned short, QString> > SortedList;
	SortedList sorted;
	for (TaxelMap::const_iterator it=taxels.begin(), end=taxels.end(); it!=end; ++it)
		sorted.insert(std::make_pair(it->channel, it.key()));

	// write mapping (sorted)
	unsigned int iWritten=0;
	for (SortedList::const_iterator it=sorted.begin(), end=sorted.end(); it!=end; ++it) {
		ts << it->second << "=" << it->first << endl;
		++iWritten;
	}
	if (iWritten > NO_TAXELS / 2) return;

	// write all other node names
	ts << endl;
	for (PathList::const_iterator it=allNodes.begin(), end=allNodes.end();
	     it != end; ++it) {
		const QString &key = it->first;
		if (key.isEmpty() || taxels.find(key) != taxels.end()) continue;
		ts << key << "=" << endl;
	}
}

void GloveWidget::edit_mapping(QString node, int channel)
{
	if (node.isEmpty()) return;
	MappingDialog dlg(node, channel, this);
	if (dlg.exec() != QDialog::Accepted) return;

	QString newName = dlg.name().trimmed();
	if (node != dlg.name() && !newName.isEmpty()) {
		// change id of path in QDomDocument
		int idx = findPathNodeIndex(node); assert(idx >= 0 && idx < allNodes.size());
		QDomNode path = allNodes[idx].second;
		QDomNamedNodeMap qmap = path.attributes();
		QDomNode nid = qmap.namedItem("id");
		nid.setNodeValue(newName);

		// change TaxelMapping: search for entry, reinsert, remove
		TaxelMap::iterator it = taxels.find(node);
		if (it != taxels.end()) {
			taxels.insert(newName, it.value());
			taxels.erase(it);
		}
		// changing allNodes is simpler: we simply change the name
		allNodes[idx].first = newName;
		update_svg();
	}
	if (dlg.channel() < 0) unassign(node);
	else assign(node, dlg.channel());
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
		for (PathList::const_iterator it=allNodes.begin(), end=allNodes.end();
		     it != end; ++it) {
			const QString &sName = it->first;
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

void GloveWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
	if (event->button() == Qt::LeftButton) {
		pair<QString, int> info = pathAt(viewTransform.inverted().map(event->pos()));
		edit_mapping(info.first, info.second);
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
	for (PathList::const_iterator it=allNodes.begin(), end=allNodes.end();
	     it != end; ++it) {
		const QString &sName = it->first;
		if (sName.isEmpty()) continue;
		QMatrix m = qSvgRendererPtr->matrixForElement(sName);
		QRectF bounds = m.mapRect(qSvgRendererPtr->boundsOnElement(sName));
		if (bounds.contains(p)) return make_pair(sName, -1);
	}
	return make_pair(QString(), -1);
}

bool GloveWidget::event(QEvent *event)
{
	if (event->type() == QEvent::ToolTip) {
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
	qSvgRendererPtr->load(qDomDocPtr->toByteArray(-1));
	update();
}

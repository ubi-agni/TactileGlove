#include "GloveWidget.h"
#include "MappingDialog.h"
#include "ColorMap.h"

#include <QtSvg>
#include <QDomDocument>
#include <QAction>
#include <stdexcept>
#include <assert.h>
#include <iostream>

using namespace std;

static const QString fillKey="fill:#";

static QString getLabel(int channel, const QString &id, bool showChannel=true, bool showID=true) {
	QString result;
	if (showChannel) result.setNum(channel+1);
	if (showChannel && showID) result.append(": ");
	if (showID) result.append(id);
	return result;
}

GloveWidget::GloveWidget(size_t noTaxels, const QString &sLayout,
                         const TaxelMapping &mapping, QWidget *parent)
   : QWidget(parent), bDirtyDoc(false), bDirtyMapping(false),
     numNoTaxelNodes(0), bMonitorTaxel(false), colorMap(0), fMin(0), fMax(4095)
{
	setupUi(this);
	data.resize(noTaxels);
	accumulated.resize(noTaxels);

	qDomDocPtr = new QDomDocument (sLayout);
	QFile file(sLayout);
	if (!file.exists()) file.setFileName(QString(":%1.svg").arg(sLayout));
	if (!file.open(QIODevice::ReadOnly))
		throw std::runtime_error("failed to open taxel layout: " + sLayout.toStdString());

	QString errorMsg;
	if (!qDomDocPtr->setContent(&file, &errorMsg)) {
		file.close();
		throw std::runtime_error(errorMsg.toStdString());
	}
	file.close();

	connect(actConfMap, SIGNAL(triggered()), this, SLOT(configureMapping()));
	connect(actSaveMapping, SIGNAL(triggered()), this, SLOT(saveMapping()));
	connect(actSaveSVG, SIGNAL(triggered()), this, SLOT(saveSVG()));

	QAction* separator=new QAction(this); separator->setSeparator(true);
	this->_optionActions << actShowChannels << actShowIDs << actShowAllIDs;
	this->_optionActions << separator << actConfMap;
	this->_fileActions << actSaveMapping << actSaveSVG;
	this->addActions(_optionActions);
	this->addActions(_fileActions);

	// retrieve mapping from all node names to their style attribute items
	QDomNodeList paths = qDomDocPtr->documentElement().elementsByTagName("path");
	for (int i=0; i<paths.count(); ++i) {
		QDomNode path = paths.at(i);
		QDomNamedNodeMap qmap = path.attributes();
		QString name = qmap.namedItem("id").nodeValue();
		QDomNode  sn = qmap.namedItem("style");
		if (name.isEmpty() || sn.isNull()) continue;
		numNoTaxelNodes += name.startsWith("path");
		allNodes.push_front(make_pair(name, path));
	}

	// realize TaxelMapping
	for (TaxelMapping::const_iterator it=mapping.begin(), end=mapping.end();
	     it!=end; ++it) {
		QString sName = QString::fromStdString(it->first);
		if (!assign(sName, it->second))
			cerr << "couldn't find a node named " << it->first << endl;
	}
	bDirtyMapping = false;
	actConfMap->setEnabled(allNodes.size() - numNoTaxelNodes > taxels.size());

	qSvgRendererPtr = new QSvgRenderer (this);
	resetData();

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
	styleString = styleNode.nodeValue();
	iFillStart = styleString.indexOf(fillKey);
	if (iFillStart == -1) {
		// fillKey missing for node: add ourselves
		iFillStart = styleString.length() + 1 + fillKey.length();
		styleString.append(";fill:#ffffff");
	} else iFillStart += fillKey.length();
}

bool GloveWidget::assign(const QString &sName, int idx) {
	if (idx < 0) {
		if (taxels.remove(sName) > 0) bDirtyMapping = true;
		return true;
	}

	const QDomNode &styleNode = findStyleNode(sName);
	if (styleNode.isNull()) return false;

	if (idx < 0 || idx >= data.size())
		throw std::runtime_error(QString("invalid taxel channel %1=%2")
	                            .arg(sName).arg(idx).toStdString());

	taxels.insert(sName, TaxelInfo(idx, styleNode));
	bDirtyMapping = true;
	return true;
}

void GloveWidget::saveSVG()
{
	QString sFileName = QFileDialog::getSaveFileName(0, "save sensor layout",
	                                                 qDomDocPtr->nodeValue(), "*.svg");
	if (sFileName.isNull()) return;

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
	bDirtyDoc = false;
}

void GloveWidget::saveMapping()
{
	typedef std::map<QString, std::pair<QString, void (GloveWidget::*)(QTextStream&)> > FilterMap;
	static FilterMap filterMap;
	static QString sFilters;
	static FilterMap::const_iterator defaultFilter;
	if (filterMap.empty()) {
		filterMap[tr("mapping configs (*.cfg)")] = make_pair(".cfg", &GloveWidget::saveMappingCfg);
		defaultFilter = filterMap.begin();
		filterMap[tr("xacro configs (*.xacro)")] = make_pair(".xacro", &GloveWidget::saveMappingXacro);
		for (FilterMap::const_iterator it=filterMap.begin(), end=filterMap.end();
		     it != end; ++it) {
			if (!sFilters.isEmpty()) sFilters.append(";;");
			sFilters.append(it->first);
		}
	}

	// choose default filter from current file name
	QString selectedFilter = defaultFilter->first;
	for (FilterMap::const_iterator it=filterMap.begin(), end=filterMap.end(); it != end; ++it) {
		if (sMappingFile.endsWith(it->second.first))
			selectedFilter = it->first;
	}

	// exec file dialog
	QString sFileName = QFileDialog::getSaveFileName(0, "save taxel mapping",
	                                                 sMappingFile, sFilters, &selectedFilter);
	if (sFileName.isNull()) return;

	// which filter was choosen?
	FilterMap::const_iterator chosenFilter = filterMap.find(selectedFilter);
	if (chosenFilter == filterMap.end()) chosenFilter = defaultFilter;

	// append default extension from selected filter
	QFileInfo fi(sFileName);
	if (fi.suffix().isEmpty()) sFileName.append(chosenFilter->second.first);

	// save name of mapping file for next saving attempt
	sMappingFile = sFileName;

	QFile file(sFileName);
	if (!file.open(QFile::WriteOnly | QFile::Truncate)) {
		QMessageBox::warning(this, "save taxel mapping",
		                     QString("Failed to open file for writing:\n%1").arg(sFileName));
		return;
	}

	QTextStream ts(&file);
	(this->*chosenFilter->second.second)(ts);
}

void GloveWidget::saveMappingCfg(QTextStream &ts) {
	// write mapping (sorted by taxel name)
	unsigned int iWritten=0;
	for (TaxelMap::const_iterator it=taxels.begin(), end=taxels.end(); it!=end; ++it) {
		ts << it.key() << "=" << it->channel << endl;
		++iWritten;
	}
	bDirtyMapping = false;
	if (iWritten > data.size() / 2) return;

	// write all other node names
	ts << endl;
	for (PathList::const_iterator it=allNodes.begin(), end=allNodes.end();
	     it != end; ++it) {
		const QString &key = it->first;
		if (key.isEmpty() || getTaxelChannel(key) != -1) continue;
		ts << key << "=" << endl;
	}
}

void GloveWidget::saveMappingXacro(QTextStream &ts) {
	ts << "<tactile_mapping xmlns:xacro=\"http://ros.org/wiki/xacro\">" << endl;
	ts << "<xacro:property name=\"tactMap\" value=\"{" << endl;
	for (TaxelMap::const_iterator it=taxels.begin(), end=taxels.end(); it!=end; ++it)
		ts << "'" << it.key() << "'" << ":" << it->channel << "," << endl;
	ts << "}\" />" << endl;
	ts << "</tactile_mapping>" << endl;
}

bool GloveWidget::canClose()
{
	if (bDirtyDoc || bDirtyMapping) {
		QMessageBox::StandardButton result
		      = QMessageBox::warning(this, "Unsaved changes",
		                             "You have unsaved changes. Close anyway?",
		                             QMessageBox::Save | QMessageBox::Ok | QMessageBox::Cancel,
		                             QMessageBox::Cancel);
		if (result == QMessageBox::Save) {
			if (bDirtyDoc) saveSVG();
			if (bDirtyMapping) saveMapping();
		}
		if (result == QMessageBox::Cancel)
			return false;
	}
	return true;
}

void GloveWidget::editMapping(QString node, int channel, MappingDialog* dlg)
{
	static const QColor highlightColor("blue");
	if (node.isEmpty()) return;
	QString oldStyle=highlight(node, highlightColor);

	bool bOwnDialog = false;
	if (!dlg) {
		dlg = new MappingDialog(this);
		bOwnDialog = true;
	}
	dlg->init(node, channel, data.size(), getUnassignedChannels());
	connect(this, SIGNAL(pushedTaxel(int)), dlg, SLOT(setChannel(int)));
	setMonitorEnabled(channel < 0);

	int res = dlg->exec(); setMonitorEnabled(false);
	restore(node, oldStyle);

	if (res != QDialog::Accepted) return;

	QString newName = dlg->name().trimmed();
	if (node != dlg->name() && !newName.isEmpty()) {
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
		allNodes[idx].first = node = newName;
		updateSVG();
		bDirtyDoc = true;
		if (newName.startsWith("path") ^ node.startsWith("path"))
			numNoTaxelNodes += newName.startsWith("path") ? 1 : -1;
	}
	if (getTaxelChannel(node) != dlg->channel())
		assign(node, dlg->channel());

	actConfMap->setEnabled(allNodes.size() - numNoTaxelNodes > taxels.size());
	if (bOwnDialog) dlg->deleteLater();
}

void GloveWidget::setMonitorEnabled(bool bEnable)
{
	bMonitorTaxel = bEnable;
	std::fill(accumulated.begin(), accumulated.end(), 0);
}

void GloveWidget::configureMapping()
{
	MappingDialog* dlg = new MappingDialog(this);
	QPoint         pos;

	bCancelConfigure = false;
	QPushButton *btn = dlg->addButton(tr("&Abort"), QDialogButtonBox::DestructiveRole);
	connect(btn, SIGNAL(clicked()), this, SLOT(setCancelConfigure()));
	connect(btn, SIGNAL(clicked()), dlg, SLOT(reject()));

	// display channel numbers, but not the names
	bool bShowChannels = actShowChannels->isChecked();
	bool bShowNames = actShowIDs->isChecked();
	bool bShowAllNames = actShowAllIDs->isChecked();
	actShowAllIDs->setChecked(false);
	actShowChannels->setChecked(true);
	actShowIDs->setChecked(false);

	for (PathList::const_iterator it=allNodes.begin(), end=allNodes.end();
	     !bCancelConfigure && it != end; ++it) {
		// ignore nodes named path*
		if (it->first.startsWith("path")) continue;
		// and nodes already assigned
		if (getTaxelChannel(it->first) >= 0) continue;

		editMapping(it->first, -1, dlg);
		pos = dlg->pos();
		dlg->show();
		dlg->move(pos);
	}
	dlg->deleteLater();

	// restore channel/name display
	actShowChannels->setChecked(bShowChannels);
	actShowIDs->setChecked(bShowNames);
	actShowAllIDs->setChecked(bShowAllNames);
}

void GloveWidget::setCancelConfigure(bool bCancel)
{
	bCancelConfigure = bCancel;
}

QSize GloveWidget::sizeHint() const
{
	return qSvgRendererPtr->defaultSize();
}

const QList<QAction*>& GloveWidget::fileActions() const
{
	return _fileActions;
}

const QList<QAction*>& GloveWidget::optionActions() const
{
	return _optionActions;
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
			QString label = getLabel(it->channel, sName,
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
		QString sName = pathAt(viewTransform.inverted().map(event->pos()));
		editMapping(sName, getTaxelChannel(sName));
	}
}

QString GloveWidget::pathAt(const QPoint &p) {
	for (PathList::const_iterator it=allNodes.begin(), end=allNodes.end();
	     it != end; ++it) {
		const QString &sName = it->first;
		if (sName.isEmpty()) continue;
		QMatrix m = qSvgRendererPtr->matrixForElement(sName);
		QRectF bounds = m.mapRect(qSvgRendererPtr->boundsOnElement(sName));
		if (bounds.contains(p)) return sName;
	}
	return QString();
}

int GloveWidget::getTaxelChannel(const QString &sName) const
{
	if (sName.isEmpty()) return -1;
	TaxelMap::const_iterator it = taxels.find(sName);
	if (it == taxels.end()) return -1;
	return it->channel;
}

QList<unsigned int> GloveWidget::getUnassignedChannels() const
{
	QList<unsigned int> unassigned;
	for (size_t i=0, end=data.size(); i < end; ++i)
		unassigned.append(i);

	for (TaxelMap::const_iterator it = taxels.begin(), end = taxels.end();
	     it != end; ++it)
		unassigned.removeOne(it->channel);
	return unassigned;
}

bool GloveWidget::event(QEvent *event)
{
	if (event->type() == QEvent::ToolTip) {
		QHelpEvent *helpEvent = static_cast<QHelpEvent *>(event);
		QString sName = pathAt(viewTransform.inverted().map(helpEvent->pos()));
		if (!sName.isEmpty()) {
			int channel = getTaxelChannel(sName);
			QToolTip::showText(helpEvent->globalPos(),
			                   getLabel(channel, sName, channel >= 0, true));
		} else {
			QToolTip::hideText();
			event->ignore();
		}
		return true;
	}
	return QWidget::event(event);
}

void GloveWidget::monitorTaxels(const TactileData &data) {
	assert(accumulated.size() == data.size());

	unsigned long valFirst=0, valSecond=0;
	int idxFirst = -1, idxSecond = -1;

	TactileData::const_iterator d = data.begin();
	for (std::vector<unsigned long>::iterator acc = accumulated.begin(), end= accumulated.end();
	     acc != end; ++acc, ++d) {
		*acc += *d;
		if (*acc > valFirst) {
			valSecond = valFirst; valFirst = *acc;
			idxSecond = idxFirst; idxFirst = acc-accumulated.begin();
		}
	}
	if (valFirst > 5 * valSecond && valFirst > 1000) {
		emit pushedTaxel(idxFirst);
		setMonitorEnabled(false);
	}
}

void GloveWidget::updateData(const TactileData &data,
                             float fMin, float fMax, ColorMap *colorMap)
{
	assert(data.size() == this->data.size());

	bool bDirty = false;
	for (TactileData::const_iterator
	     me=this->data.begin(), end=this->data.end(), other=data.begin();
	     me != end; ++me, ++other) {
		if (*me != *other) {
			bDirty = true;
			break;
		}
	}
	if (bMonitorTaxel) monitorTaxels(data);
	if (!bDirty) return;

	this->data = data;
	this->fMin = fMin; this->fMax = fMax;
	this->colorMap = colorMap;
	updateTaxels();
}

void GloveWidget::resetData()
{
	std::fill(data.begin(), data.end(), 0);
	if (!colorMap) updateSVG(); else updateTaxels();
}

static QString hexRGB(const QColor &color) {
#if (0 && QT_VERSION >= QT_VERSION_CHECK(5, 2, 0))
	return color.name(QColor::HexRgb).mid(1));
#else
	unsigned int col = (color.red() << 16) | (color.green() << 8) | color.blue();
	return QString("%1").arg(col, 6, 16, QLatin1Char('0'));
#endif
}

QString GloveWidget::highlight(const QString &sName, const QColor &color)
{
	QDomNode styleNode = findStyleNode(sName);
	if (styleNode.isNull()) return QString();

	QString oldStyle = styleNode.nodeValue();
	highlighted.insert(sName);
	styleNode.setNodeValue(fillKey + hexRGB(color));
	updateSVG();
	return oldStyle;
}
void GloveWidget::restore(const QString &sName, const QString &style) {
	if (sName.isEmpty() || style.isEmpty()) return;
	QDomNode styleNode = findStyleNode(sName);
	if (styleNode.isNull()) return;

	highlighted.remove(sName);
	styleNode.setNodeValue(style);
	updateSVG();
}

void GloveWidget::updateTaxels()
{
	static const QString fmt("%1");
	for (TaxelMap::iterator it=taxels.begin(), end=taxels.end(); it!=end; ++it) {
		if (highlighted.contains(it.key())) continue;

		QColor color = colorMap->map(data[it->channel], fMin, fMax);
		// replace color string in style string
		it->styleString.replace(it->iFillStart, 6, hexRGB(color));
		it->styleNode.setNodeValue(it->styleString);
	}
	updateSVG();
}
void GloveWidget::updateSVG()
{
	qSvgRendererPtr->load(qDomDocPtr->toByteArray(-1));
	update();
}

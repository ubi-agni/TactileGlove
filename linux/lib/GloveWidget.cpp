#include "GloveWidget.h"

#include <QtSvg>
#include <QDomDocument>
#include <QAction>
#include <stdexcept>
#include <assert.h>

static const QString fillKey = "fill:#";

static QString getLabel(int channel, const QString &id, bool showChannel = true, bool showName = true)
{
	QString result;
	if (showChannel)
		result.setNum(channel + 1);
	if (showChannel && showName)
		result.append(": ");
	if (showName)
		result.append(id);
	return result;
}

GloveWidget::GloveWidget(const QString &sLayout, bool bMirror, QWidget *parent)
  : QWidget(parent)
  , bDirtyDoc(false)
  , numTaxelNodes(0)
  , numAssigned(0)
  , bShowChannels(false)
  , bShowNames(false)
  , bShowAllNames(false)
  , bMirror(bMirror)
{
	qDomDocPtr = new QDomDocument(sLayout);
	QFile file(sLayout);
	if (!file.exists())
		file.setFileName(QString(":%1.svg").arg(sLayout));
	if (!file.open(QIODevice::ReadOnly))
		throw std::runtime_error("failed to open taxel layout: " + sLayout.toStdString());

	QString errorMsg;
	if (!qDomDocPtr->setContent(&file, &errorMsg)) {
		file.close();
		throw std::runtime_error(errorMsg.toStdString());
	}
	file.close();

	// retrieve mapping from all node names to their style attribute items
	QDomNodeList paths = qDomDocPtr->documentElement().elementsByTagName("path");
	for (int i = 0; i < paths.count(); ++i) {
		QDomNode path = paths.at(i);
		QDomNamedNodeMap qmap = path.attributes();
		QString name = qmap.namedItem("id").nodeValue();
		QDomNode sn = qmap.namedItem("style");
		if (name.isEmpty() || sn.isNull())
			continue;

		if (!name.startsWith("path"))
			++numTaxelNodes;
		allNodes.push_front(TaxelInfo(name, path, sn));
	}
	std::sort(allNodes.begin(), allNodes.end(), [](const TaxelInfo &l, const TaxelInfo &r) { return l.name < r.name; });
	emit unAssignedTaxels(numAssigned < numTaxelNodes);

	qSvgRendererPtr = new QSvgRenderer(this);

	QSizePolicy sp(QSizePolicy::Preferred, QSizePolicy::Preferred);
	sp.setHeightForWidth(true);
	setSizePolicy(sp);
}

unsigned int GloveWidget::numNodes() const
{
	return allNodes.size();
}

int GloveWidget::findPathNodeIndex(const QString &sName) const
{
	int idx = 0;
	for (PathList::const_iterator it = allNodes.begin(), end = allNodes.end(); it != end; ++it, ++idx)
		if (it->name == sName)
			return idx;
	return -1;  // not found
}

QDomNode GloveWidget::findStyleNode(const QString &sName) const
{
	int idx = findPathNodeIndex(sName);
	if (idx < 0)
		return QDomNode();
	return allNodes[idx].styleNode;
}

const QString &GloveWidget::getNodeName(unsigned int nodeIdx) const
{
	return allNodes[nodeIdx].name;
}

void GloveWidget::setNodeName(unsigned int nodeIdx, const QString &name)
{
	TaxelInfo info = allNodes[nodeIdx];
	const QString &oldName = info.name;
	info.name = name;
	QDomNamedNodeMap qmap = info.pathNode.attributes();
	QDomNode nid = qmap.namedItem("id");
	nid.setNodeValue(name);
	bDirtyDoc = true;

	if (name.startsWith("path") ^ oldName.startsWith("path")) {
		numTaxelNodes += name.startsWith("path") ? -1 : 1;
		emit unAssignedTaxels(numAssigned < numTaxelNodes);
	}
}

int GloveWidget::nodeAt(const QPoint &p)
{
	for (PathList::const_iterator it = allNodes.begin(), end = allNodes.end(); it != end; ++it) {
		if (it->name.isEmpty())
			continue;
		QMatrix m = qSvgRendererPtr->matrixForElement(it->name);
		QRectF bounds = m.mapRect(qSvgRendererPtr->boundsOnElement(it->name));
		if (bounds.contains(p))
			return it - allNodes.begin();
	}
	return -1;
}

GloveWidget::TaxelInfo::TaxelInfo(const QString &name, const QDomNode &pathNode, const QDomNode &styleNode)
  : pathNode(pathNode), name(name), styleNode(styleNode), channel(-1)
{
	styleString = styleNode.nodeValue();
	iFillStart = styleString.indexOf(fillKey);
	if (iFillStart == -1) {
		// fillKey missing for node: add ourselves
		iFillStart = styleString.length() + 1 + fillKey.length();
		styleString.append(";fill:#ffffff");
	} else
		iFillStart += fillKey.length();
}

void GloveWidget::setChannel(unsigned int nodeIdx, int channelIdx)
{
	int oldChannel = allNodes[nodeIdx].channel;
	allNodes[nodeIdx].channel = channelIdx;
	if (oldChannel != channelIdx) {
		numAssigned += channelIdx < 0 ? -1 : +1;
		emit unAssignedTaxels(numAssigned < numTaxelNodes);
	}
}

void GloveWidget::saveSVG()
{
	QString sFileName = QFileDialog::getSaveFileName(nullptr, "save sensor layout", qDomDocPtr->nodeValue(), "*.svg");
	if (sFileName.isNull())
		return;

	QFileInfo info(sFileName);
	sFileName.remove(sFileName.size() - 1 - info.completeSuffix().size(), sFileName.size());
	sFileName.append(".svg");

	QFile file(sFileName);
	if (!file.open(QFile::WriteOnly | QFile::Truncate)) {
		QMessageBox::warning(this, "save sensor layout", QString("Failed to open file for writing:\n%1").arg(sFileName));
		return;
	}

	QTextStream ts(&file);
	qDomDocPtr->save(ts, 2);
	qDomDocPtr->setNodeValue(sFileName);
	bDirtyDoc = false;
}

void GloveWidget::setShowChannels(bool bShow)
{
	bShowChannels = bShow;
	update();
}

void GloveWidget::setShowNames(bool bShow)
{
	bShowNames = bShow;
	update();
}

void GloveWidget::setShowAllNames(bool bShow)
{
	bShowAllNames = bShow;
	update();
}

bool GloveWidget::saveMappingCfg(const QString &sFileName, const QString &sMappingName)
{
	QSettings ini(sFileName, QSettings::IniFormat);

	if (!sMappingName.isEmpty())
		ini.beginGroup(sMappingName);
	ini.remove("");  // remove all keys in current group

	for (const auto &node : allNodes) {
		if (node.channel < 0)
			continue;
		ini.setValue(node.name, node.channel);
	}
	ini.sync();
	return true;
}

bool GloveWidget::saveMappingYAML(const QString &sFileName, const QString & /*sMappingName*/)
{
	QFile file(sFileName);
	if (!file.open(QFile::WriteOnly | QFile::Truncate))
		return false;

	QTextStream ts(&file);
	for (const auto &node : allNodes) {
		if (node.channel < 0)
			continue;
		ts << node.name << ":" << node.channel << endl;
	}
	return true;
}

bool GloveWidget::canClose()
{
	if (bDirtyDoc) {
		QMessageBox::StandardButton result =
		    QMessageBox::warning(this, "Unsaved name changes", "You have unsaved name changes. Close anyway?",
		                         QMessageBox::Save | QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
		if (result == QMessageBox::Save)
			saveSVG();
		if (result == QMessageBox::Cancel)
			return false;
	}
	return true;
}

QSize GloveWidget::sizeHint() const
{
	return qSvgRendererPtr->defaultSize();
}

void GloveWidget::paintEvent(QPaintEvent * /*event*/)
{
	QPainter painter(this);
	painter.setRenderHint(QPainter::HighQualityAntialiasing, false);

	// setup correct scaling
	const QSize &svgSize = qSvgRendererPtr->defaultSize();
	QSize renderSize(svgSize);
	renderSize.scale(painter.viewport().size(), Qt::KeepAspectRatio);
	painter.setWindow(0, 0, svgSize.width(), svgSize.height());  // logical coordinates are fixed
	painter.setViewport(0, 0, renderSize.width(), renderSize.height());

	// flipping transform for SVG drawing
	QMatrix tf;
	if (bMirror) {
		tf.translate(svgSize.width(), 0);
		tf.scale(-1, 1);
	}
	painter.setMatrix(tf);
	viewTransform = painter.combinedTransform();
	qSvgRendererPtr->render(&painter, painter.window());

	QFont font = painter.font();
	font.setPointSize(8);
	painter.setFont(font);

	// reset transform for text drawing
	painter.setMatrix(QMatrix());
	// show IDs/channels of taxels
	for (const auto &node : allNodes) {
		if (!bShowAllNames && node.channel < 0)
			continue;
		QMatrix m = qSvgRendererPtr->matrixForElement(node.name) * tf;
		QRectF bounds = m.mapRect(qSvgRendererPtr->boundsOnElement(node.name));
		QString label = getLabel(node.channel, node.name, bShowChannels && !bShowAllNames, bShowNames || bShowAllNames);
		painter.setPen(Qt::black);
		painter.drawText(bounds.translated(1, 1), Qt::AlignCenter, label);
		painter.setPen(Qt::white);
		painter.drawText(bounds, Qt::AlignCenter, label);
	}
}

void GloveWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
	if (event->button() == Qt::LeftButton) {
		int idx = nodeAt(viewTransform.inverted().map(event->pos()));
		if (idx < 0)
			return;  // no node found
		emit doubleClickedNode(idx);
	}
}

bool GloveWidget::event(QEvent *event)
{
	if (event->type() == QEvent::ToolTip) {
		QHelpEvent *helpEvent = static_cast<QHelpEvent *>(event);
		int nodeIdx = nodeAt(viewTransform.inverted().map(helpEvent->pos()));
		if (nodeIdx >= 0) {
			TaxelInfo i = allNodes[nodeIdx];
			QToolTip::showText(helpEvent->globalPos(), getLabel(i.channel, i.name, i.channel >= 0, true));
		} else {
			QToolTip::hideText();
			event->ignore();
		}
		return true;
	}
	return QWidget::event(event);
}

static QString hexRGB(const QColor &color)
{
#if (0 && QT_VERSION >= QT_VERSION_CHECK(5, 2, 0))
	return color.name(QColor::HexRgb).mid(1));
#else
	unsigned int col = (color.red() << 16) | (color.green() << 8) | color.blue();
	return QString("%1").arg(col, 6, 16, QLatin1Char('0'));
#endif
}

QString GloveWidget::highlight(unsigned int nodeIdx, const QColor &color)
{
	TaxelInfo i = allNodes[nodeIdx];
	QString oldStyle = i.styleNode.nodeValue();
	i.styleNode.setNodeValue(fillKey + hexRGB(color));
	updateSVG();
	return oldStyle;
}
void GloveWidget::restore(unsigned int nodeIdx, const QString &style)
{
	allNodes[nodeIdx].styleNode.setNodeValue(style);
	updateSVG();
}

void GloveWidget::updateColor(unsigned int nodeIdx, const QColor &color)
{
	static const QString fmt("%1");
	// replace color string in style string
	TaxelInfo &i = allNodes[nodeIdx];
	i.styleString.replace(i.iFillStart, 6, hexRGB(color));
	i.styleNode.setNodeValue(i.styleString);
}

void GloveWidget::updateSVG()
{
	qSvgRendererPtr->load(qDomDocPtr->toByteArray(-1));
	update();
}

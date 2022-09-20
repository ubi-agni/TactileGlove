#pragma once

#include <QWidget>
#include <QDomNode>

class QDomDocument;
class QDomNode;
class QSvgRenderer;
class MappingDialog;
class ColorMap;

/** Provides a means to visualize the TactileGlove from an SVG
 *  and to change coloring of the individual taxels
 */
class GloveWidget : public QWidget
{
	Q_OBJECT
public:
	explicit GloveWidget(const QString &sLayout, bool bMirror = false, QWidget *parent = nullptr);
	QSize sizeHint() const override;

	/// return number of all SVG nodes
	unsigned int numNodes() const;
	/// find a path node in allNodes from sName
	int findPathNodeIndex(const QString &sName) const;
	/// get name of indexed node
	const QString &getNodeName(unsigned int nodeIdx) const;
	/// set name of indexed node
	void setNodeName(unsigned int nodeIdx, const QString &name);
	/// establish the mapping from nodeIdx to channelIdx
	void setChannel(unsigned int nodeIdx, int idx);

	/// highlight indexed node with given color and return previous style string
	QString highlight(unsigned int nodeIdx, const QColor &color);
	/// restore style string for indexed node
	void restore(unsigned int nodeIdx, const QString &style);

	/// update color of indexed node
	void updateColor(unsigned int nodeIdx, const QColor &color);
	/// reload the SVG and update display
	void updateSVG();

	/// saving to cfg or xacro
	bool saveMappingCfg(const QString &sFileName, const QString &sMappingName = QString());
	bool saveMappingYAML(const QString &sFileName, const QString &sMappingName = QString());
	/// check for unsaved changes to SVG
	bool canClose();

public slots:
	void saveSVG();
	void setShowChannels(bool bShow);
	void setShowNames(bool bShow);
	void setShowAllNames(bool bShow);

signals:
	void unAssignedTaxels(bool bHaveUnassigned);
	void doubleClickedNode(unsigned int index);

private:
	/// find the style attribute node of a path node in allNodes from sName
	QDomNode findStyleNode(const QString &sName) const;
	/// find index of path node in allNodes at point (or -1 if not found)
	int nodeAt(const QPoint &p);

	bool event(QEvent *event) override;
	void paintEvent(QPaintEvent *event) override;
	void mouseDoubleClickEvent(QMouseEvent *event) override;

private:
	struct TaxelInfo
	{
		TaxelInfo(const QString &name, const QDomNode &pathNode, const QDomNode &styleNode);

		QDomNode pathNode;
		QString name;
		QDomNode styleNode;
		QString styleString;
		int iFillStart;
		short channel;  // -1 if unassigned
	};

	QDomDocument *qDomDocPtr;
	bool bDirtyDoc;
	QSvgRenderer *qSvgRendererPtr;

	// list of all DOM nodes with their name
	using PathList = QList<TaxelInfo>;
	PathList allNodes;

	unsigned int numTaxelNodes, numAssigned;
	bool bShowChannels, bShowNames, bShowAllNames;
	QTransform viewTransform;  // painter's transform, used for mouse mapping
	bool bMirror;
};

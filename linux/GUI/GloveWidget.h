#pragma once

#include "TaxelMapping.h"
#include <QWidget>
#include <QDomNode>
#include <QMutex>
#include <QVector>
#include <QMap>
#include <QSet>

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

	const QList<QAction*>& fileActions() const;
	const QList<QAction*>& optionActions() const;

public slots:
	/// update internal data buffer and trigger updateSVG() on changes
	void updateData(unsigned short* data);
	/// reset SVG display
	void resetData();

	QString highlight(const QString &sName, const QColor &color=QColor("red"));
	void restore(const QString &sName, const QString &style);

	void saveSVG();
	void saveMapping();
	void editMapping(QString node, int channel);
	/// enable peak monitoring, see monitorTaxels()
	void setMonitorEnabled(bool bEnable=true);
	/// configure all unassigned taxels
	void configureMapping();
	/// do we have changes not yet saved?
	bool canClose();

signals:
	void pushedTaxel(int);

private:
	/// find a path node in allNodes from sName
	int findPathNodeIndex(const QString &sName) const;
	/// find the style attribute node of a path node in allNodes from sName
	QDomNode findStyleNode(const QString &sName) const;
	/// find name of path node in allNodes at point (or QString() if not found)
	QString pathAt(const QPoint &p);
	/// find the channel index associated to sName (or -1 if not found)
	int getTaxelChannel(const QString &sName) const;

	/// establish the mapping from channel idx to taxel sName
	bool assign(const QString &sName, int idx);
	/// remove channel association from taxel sName
	void unassign(const QString &sName);

	bool event(QEvent *event);
	void paintEvent(QPaintEvent *event);
	void mouseDoubleClickEvent(QMouseEvent *event);

	/// look for a peak in accumulated taxel data and signal its index
	void monitorTaxels(unsigned short *data);
	/// update the SVG with new colors
	void updateTaxels();
	/// reload the SVG and update display
	void updateSVG();

private:
	struct TaxelInfo {
		TaxelInfo (unsigned short idx, const QDomNode&node);

		unsigned short channel;
		QDomNode styleNode;
		QString  styleString;
		unsigned short iFillStart;
	};

	QDomDocument  *qDomDocPtr;
	bool           bDirtyDoc;
	QSvgRenderer  *qSvgRendererPtr;
	unsigned short data[NO_TAXELS];
	unsigned long  accumulated[NO_TAXELS];
	bool           bMonitorTaxel;

	typedef QMap<QString, TaxelInfo> TaxelMap;
	TaxelMap       taxels;
	bool           bDirtyMapping;
	typedef QList<std::pair<QString, QDomNode> > PathList;
	PathList       allNodes;
	unsigned int   numNoTaxelNodes;
	QSet<QString>  highlighted;

	QAction       *actShowChannels;
	QAction       *actShowIDs;
	QAction       *actShowAllIDs;
	QAction       *actConfMap;
	QAction       *actSaveMapping;
	QAction       *actSaveSVG;
	QList<QAction*> _fileActions;
	QList<QAction*> _optionActions;

	QTransform     viewTransform;
};


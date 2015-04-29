#pragma once

#include "TaxelMapping.h"
#include "InputInterface.h"
#include "ui_GloveWidget.h"
#include "ColorMap.h"

#include <QWidget>
#include <QDomNode>
#include <QMutex>
#include <QVector>
#include <QMap>
#include <QSet>

class QDomDocument;
class QDomNode;
class QSvgRenderer;
class MappingDialog;

class GloveWidget : public QWidget, private Ui::GloveWidget
{
	Q_OBJECT
public:
	typedef tactile::InputInterface::data_vector data_vector;

	explicit GloveWidget(size_t noTaxels, const QString& sLayout,
	                     const TaxelMapping& mapping,
	                     QWidget *parent = 0);
	QSize sizeHint() const;

	const QList<QAction*>& fileActions() const;
	const QList<QAction*>& optionActions() const;

public slots:
	/// update internal data buffer and trigger updateSVG() on changes
	void updateData(const data_vector &data);
	/// reset SVG display
	void resetData();

	QString highlight(const QString &sName, const QColor &color=QColor("red"));
	void restore(const QString &sName, const QString &style);

	void saveSVG();
	void saveMapping();
	void editMapping(QString node, int channel, MappingDialog *dlg=0);
	/// enable peak monitoring, see monitorTaxels()
	void setMonitorEnabled(bool bEnable=true);
	/// configure all unassigned taxels
	void configureMapping();
	void setCancelConfigure(bool bCancel=true);
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

	bool event(QEvent *event);
	void paintEvent(QPaintEvent *event);
	void mouseDoubleClickEvent(QMouseEvent *event);

	/// look for a peak in accumulated taxel data and signal its index
	void monitorTaxels(const data_vector &data);
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
	data_vector    data;
	std::vector<unsigned long> accumulated;
	bool           bMonitorTaxel;
	bool           bCancelConfigure;

	typedef QMap<QString, TaxelInfo> TaxelMap;
	TaxelMap       taxels;
	bool           bDirtyMapping;
	typedef QList<std::pair<QString, QDomNode> > PathList;
	PathList       allNodes;
	unsigned int   numNoTaxelNodes;
	QSet<QString>  highlighted;

	QString        sMappingFile;

	QList<QAction*> _fileActions;
	QList<QAction*> _optionActions;

	QTransform     viewTransform;
	ColorMap       colorMap;
};

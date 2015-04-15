#pragma once

#include "TaxelMapping.h"
#include <QWidget>
#include <QDomNode>
#include <QMutex>
#include <QVector>
#include <QMap>

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

public slots:
	void update_data(unsigned short* data);
	void reset_data();
	void save_svg();
	void save_mapping();
	void edit_mapping(QString node, int channel);

private:
	int findPathNodeIndex(const QString &sName) const;
	QDomNode findStyleNode(const QString &sName) const;
	bool assign(const QString &sName, int idx);
	void unassign(const QString &sName);

	bool event(QEvent *event);
	void paintEvent(QPaintEvent *event);
	void mouseDoubleClickEvent(QMouseEvent *event);

	void update_svg();
	std::pair<QString, int> pathAt(const QPoint &p);

private:
	struct TaxelInfo {
		TaxelInfo (unsigned short idx, const QDomNode&node);

		unsigned short channel;
		QDomNode styleNode;
		QString  styleString;
		unsigned short iFillStart;
	};

	QDomDocument  *qDomDocPtr;
	QSvgRenderer  *qSvgRendererPtr;
	unsigned short data[NO_TAXELS];

	typedef QMap<QString, TaxelInfo> TaxelMap;
	TaxelMap       taxels;
	typedef QList<std::pair<QString, QDomNode> > PathList;
	PathList       allNodes;

	QAction       *actShowChannels;
	QAction       *actShowIDs;
	QAction       *actShowAllIDs;
	QAction       *actSaveMapping;
	QAction       *actSaveSVG;
	QTransform     viewTransform;
};


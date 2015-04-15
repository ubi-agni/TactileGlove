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
	void save_mapping();
	void edit_mapping(const QString &node, int channel);

private:
	const QDomNode &find(const QString &sNode) const;
	bool assign(const QString &sName, int idx);
	void unassign(const QString &sName);

	void paintEvent(QPaintEvent *event);
	void update_svg();
	bool event(QEvent *event);
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
	typedef QMap<QString, QDomNode>  StyleNodeMap;
	StyleNodeMap   allNodes;

	QAction       *actShowChannels;
	QAction       *actShowIDs;
	QAction       *actShowAllIDs;
	QAction       *actRemap;
	QAction       *actSaveMapping;
	QTransform     viewTransform;
};


#pragma once

#include <QMainWindow>
#include <QTime>
#include <QMutex>
#include <QMap>
#include <QSet>

#include "TaxelMapping.h"
#include "InputInterface.h"
#include <tactile_filters/TactileValueArray.h>

namespace Ui {class MainWindow;}
class QComboBox;
class ColorMap;

class GloveWidget;
class MappingDialog;
class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(size_t noTaxels, QWidget *parent = 0);
	~MainWindow();

	void initJointBar(TaxelMapping &mapping);
	void initGloveWidget(const QString &layout, const TaxelMapping& mapping);

	void configSerial(const QString &sDevice);
	void configROS(const QString &sTopic);
	void configRandom();

public slots:
	void saveMapping();

private:
	void initModeComboBox(QComboBox *cb);
	void chooseMapping(tactile::TactileValue::Mode mode, ColorMap *&colorMap,
	                   float &fMin, float &fMax);
	void resetColors(const QColor &color=QColor("black"));
	void updateData(const InputInterface::data_vector &taxels);
	void updateJointBar(unsigned short value);

	void timerEvent(QTimerEvent *event);
	void closeEvent(QCloseEvent *event);

	QList<unsigned int> getUnassignedChannels() const;

private slots:
	void on_btnConnect_clicked();
	void on_btnDisconnect_clicked();

	void setTimer(int interval);
	void setLambda(double value);

	/// display mapping dialog for indexed taxel
	void editMapping(unsigned int nodeIdx);
	/// configure all unassigned taxels
	void configureMapping();
	void setCancelConfigure(bool bCancel=true);
	void resetMapDlg();

private:
	Ui::MainWindow  *ui;
	GloveWidget     *gloveWidget;
	InputInterface  *input;

	QMutex           dataMutex;
	tactile::TactileValueArray     data;
	tactile::TactileValueArray::vector_data display;

	/// mapping from node indeces to data indeces
	typedef QMap<unsigned int, unsigned int> NodeToDataMap;
	NodeToDataMap    nodeToData;
	QString          sMappingFile;
	bool             bDirtyMapping;

	// stuff needed for taxel mapping configuration
	MappingDialog      *mapDlg;
	QSet<unsigned int>  highlighted; /// highlighted node indeces (to avoid updating them)
	bool                bCancelConfigure;
	std::vector<unsigned long> accumulated;

	int              iJointIdx;

	ColorMap        *absColorMap, *relColorMap;

	QTime            lastUpdate;
	int              frameCount;
	int              timerID;
};

#pragma once

#include <QMainWindow>
#include <QTime>
#include <QMutex>
#include "TaxelMapping.h"
#include "InputInterface.h"
#include "TactileArray.h"

namespace Ui {class MainWindow;}
class QComboBox;
class ColorMap;

class GloveWidget;
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

private:
	void initModeComboBox(QComboBox *cb);
	void chooseMapping(TactileSensor::Mode mode, ColorMap *&colorMap,
	                   float &fMin, float &fMax);
	void updateData(const InputInterface::data_vector &taxels);
	void updateJointBar(unsigned short value);
	void timerEvent(QTimerEvent *event);
	void closeEvent(QCloseEvent *event);

private slots:
	void on_btnConnect_clicked();
	void on_btnDisconnect_clicked();

	void setTimer(int interval);
	void setLambda(double value);

private:
	Ui::MainWindow  *ui;
	GloveWidget     *gloveWidget;
	InputInterface  *input;

	QMutex           dataMutex;
	TactileArray     data;
	int              iJointIdx;
	TactileArray::vector_data display;
	ColorMap        *absColorMap, *relColorMap;

	QTime            lastUpdate;
	uint             frameCount;
	int              timerID;
};

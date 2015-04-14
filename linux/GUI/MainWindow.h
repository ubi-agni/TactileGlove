#pragma once

#include <QMainWindow>
#include <QTime>
#include "TaxelMapping.h"
#include "GloveWidget.h"

namespace Ui {class MainWindow;}

class InputInterface;
class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

	void initGloveWidget(const QString &layout, const TaxelMapping& mapping);
	void configSerial(const QString &sDevice);
	void configROS(const QString &sTopic);
	void configRandom();

private:
	void updateData(const unsigned short *data);
	void updateJointBar(unsigned short value);
	void timerEvent(QTimerEvent *event);

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
	float            frameData[NO_TAXELS];
	float            lambda;

	QTime            lastUpdate;
	uint             frameCount;
	int              timerID;
};

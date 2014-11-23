#pragma once

#include <QMainWindow>
#include <QTime>
#include "SerialThread.h"

namespace Ui {
class MainWindow;
}
class GloveWidget;
class SerialThread;

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

private:
	void updateData(unsigned short *data);
	void updateJointBar(unsigned short value);
	void timerEvent(QTimerEvent *event);

private slots:
	void on_btnConnect_clicked();
	void on_btnDisconnect_clicked();
	void setTimer(int interval);
	void setLambda(double value);

private:
	Ui::MainWindow *ui;
	GloveWidget    *gloveWidget;
	SerialThread   *serialThread;

	QMutex          dataMutex;
	float           frameData[NO_TAXELS];
	float           lambda;

	QTime           lastUpdate;
	uint            frameCount;
	int             timerID;
};

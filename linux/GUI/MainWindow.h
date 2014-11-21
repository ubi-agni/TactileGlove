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
	void updateJointBar(unsigned short data);

private slots:
	void on_btnConnect_clicked();
	void on_btnDisconnect_clicked();

signals:
	void jointChanged(int);

private:
	Ui::MainWindow *ui;
	GloveWidget    *gloveWidget;
	SerialThread   *serialThread;
	QTime           lastUpdate;
};

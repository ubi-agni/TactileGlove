#pragma once

#include <QMainWindow>
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

private slots:
	void on_pushButtonConnect_clicked();
	void on_pushButtonDisconnect_clicked();
	void updateJointBar(unsigned short*);

private:
	Ui::MainWindow *ui;
	GloveWidget *gsp;
	SerialThread* serialThread;
};

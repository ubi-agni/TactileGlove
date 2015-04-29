#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "GloveWidget.h"
#include "ColorMap.h"

#include "SerialThread.h"
#include "RandomInput.h"
#if HAVE_ROS
#include "ROSInput.h"
#endif
#include <math.h>
#include <boost/bind.hpp>
#include <QCloseEvent>
#include <QComboBox>

MainWindow::MainWindow(size_t noTaxels, QWidget *parent) :
   QMainWindow(parent), ui(new Ui::MainWindow), iJointIdx(-1),
   input(0), data(noTaxels), display(noTaxels),
   frameCount(0), timerID(0), gloveWidget(0),
   absColorMap(0), relColorMap(0)
{
	ui->setupUi(this);
	ui->toolBar->addWidget(ui->updateTimeSpinBox);
	ui->toolBar->addWidget(ui->lambdaSpinBox);
	ui->toolBar->addWidget(ui->modeComboBox);
	ui->toolBar->addWidget(ui->inputLineEdit);
	ui->toolBar->addWidget(ui->btnConnect);
	ui->toolBar->addWidget(ui->btnDisconnect);
	ui->btnDisconnect->setEnabled(false);
	ui->fps->hide();

	connect(ui->updateTimeSpinBox, SIGNAL(valueChanged(int)), this, SLOT(setTimer(int)));
	connect(ui->lambdaSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setLambda(double)));

	ui->lambdaSpinBox->setValue(0.5);
	data.setRangeLambda(0.9995);
	data.setReleaseDecay(0.1);
	initModeComboBox(ui->modeComboBox);

	// init color maps
	QStringList colorNames;
	absColorMap = new ColorMap;
	colorNames << "black" << "lime" << "yellow" << "red";
	absColorMap->append(colorNames);

	relColorMap = new ColorMap;
	colorNames.clear(); colorNames << "red" << "black" << "lime";
	relColorMap->append(colorNames);
}

void MainWindow::initModeComboBox(QComboBox *cb) {
	QStringList items;
	for (unsigned int m=0, e=TactileSensor::lastMode; m!=e; ++m)
		items << TactileSensor::getModeName((TactileSensor::Mode)m).c_str();
	cb->addItems(items);
	cb->setCurrentIndex (TactileSensor::relCurrent);
}

MainWindow::~MainWindow()
{
	if (input) {
		input->disconnect();
		delete input;
	}
	delete ui;
	delete absColorMap;
	delete relColorMap;
}

void MainWindow::initJointBar(TaxelMapping &mapping) {
	// do we have a joint value?
	TaxelMapping::iterator it = mapping.find("bar");
	if (it != mapping.end()) {
		iJointIdx = it->second;
		mapping.erase(it);
	} else iJointIdx = -1;

	if (iJointIdx < 0 || iJointIdx >= data.size()) {
		iJointIdx = -1;
		ui->jointBar->hide();
	} else ui->jointBar->show();
}

void MainWindow::initGloveWidget(const QString &layout, const TaxelMapping &mapping) {
	// do before creating the GloveWidget to allow for removing of the bar mapping
	initJointBar(const_cast<TaxelMapping&>(mapping));

	QMutexLocker lock(&dataMutex);
	if (gloveWidget) {
		on_btnDisconnect_clicked();
		ui->verticalLayout->removeWidget(gloveWidget);
		delete gloveWidget;
		gloveWidget = 0;
	}
	gloveWidget = new GloveWidget(data.size(), layout, mapping);
	ui->verticalLayout->insertWidget(0, gloveWidget);

	ui->menuFile->addActions(gloveWidget->fileActions());
	ui->menuOptions->addActions(gloveWidget->optionActions());
	ui->menuFile->addAction(ui->actionQuit);

	gloveWidget->setFocus();
}

void MainWindow::setTimer(int interval)
{
	if (!timerID) return;
	killTimer(timerID);
	timerID = startTimer(interval);
}

void MainWindow::setLambda(double value)
{
	data.setMeanLambda(value);
}

void MainWindow::chooseMapping(TactileSensor::Mode mode,
                               ColorMap *&colorMap, float &fMin, float &fMax) {
	switch (mode) {
	case TactileSensor::rawCurrent:
	case TactileSensor::rawMean:
	case TactileSensor::absCurrent:
	case TactileSensor::absMean:
		fMin=0; fMax=4095;
		colorMap = absColorMap;
		break;

	case TactileSensor::relCurrent:
	case TactileSensor::relMean:
		fMin=0; fMax=1;
		colorMap = absColorMap;
		break;

	case TactileSensor::relCurrentRelease:
	case TactileSensor::relMeanRelease:
		fMin=-1; fMax=1;
		colorMap = relColorMap;
		break;
	}
}

void MainWindow::timerEvent(QTimerEvent *event)
{
	if (event->timerId() != timerID) return;

	QMutexLocker lock(&dataMutex);
	int fps = -1;

	TactileSensor::Mode mode = (TactileSensor::Mode) ui->modeComboBox->currentIndex();
	data.copyValues(display.begin(), mode);

	if (lastUpdate.elapsed() > 1000) { // update framerate every 1s
		fps = roundf (float(frameCount * 1000) / lastUpdate.restart());
		frameCount = 0;
	}
	if (!gloveWidget) return;
	lock.unlock();

	ColorMap *colorMap;
	float fMin, fMax;
	chooseMapping(mode, colorMap, fMin, fMax);
	gloveWidget->updateData(display, fMin, fMax, colorMap);

	if (iJointIdx >= 0) updateJointBar(display[iJointIdx]);
	if (fps >= 0) ui->fps->setText(QString("%1 fps").arg(fps));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	if (gloveWidget && !gloveWidget->canClose())
		event->ignore();
}

void MainWindow::updateData(const InputInterface::data_vector &taxels) {
	QMutexLocker lock(&dataMutex);
	assert(taxels.size() == data.size());

	data.updateValues(taxels.begin(), taxels.end());
	++frameCount;
}

void MainWindow::updateJointBar(unsigned short value)
{
	const int min=4095;
	const int max=2000;
	const int targetRange=100;
	ui->jointBar->setValue(((value-min) * targetRange) / (max-min));
}


void MainWindow::configSerial(const QString &sDevice)
{
	ui->inputLineEdit->setText(sDevice);
	ui->inputLineEdit->setToolTip("serial device name");

	SerialThread *serial = new SerialThread(data.size());
	serial->setUpdateFunction(boost::bind(&MainWindow::updateData, this, _1));
	connect(serial, SIGNAL(statusMessage(QString,int)),
	        ui->statusBar, SLOT(showMessage(QString,int)));
	input = serial;
}

void MainWindow::configROS(const QString &sTopic)
{
#if HAVE_ROS
	ui->inputLineEdit->setText(sTopic);
	ui->inputLineEdit->setToolTip("ROS topic");

	ROSInput *rosInput = new ROSInput(data.size());
	rosInput->setUpdateFunction(boost::bind(&MainWindow::updateData, this, _1));
	connect(rosInput, SIGNAL(statusMessage(QString,int)),
	        ui->statusBar, SLOT(showMessage(QString,int)));
	input = rosInput;

	on_btnConnect_clicked(); // auto-connect to ROS topic
#endif
}

void MainWindow::configRandom()
{
	ui->verticalLayout->addWidget(ui->inputLineEdit);
	ui->inputLineEdit->hide();
	input = new RandomInput(data.size());
	input->setUpdateFunction(boost::bind(&MainWindow::updateData, this, _1));
}

void MainWindow::on_btnConnect_clicked()
{
	ui->statusBar->showMessage (QString ("Connecting..."), 2000);

	if (input->connect(ui->inputLineEdit->text())) {
		frameCount = 0; lastUpdate.start();
		timerID = startTimer (ui->updateTimeSpinBox->value());

		ui->btnConnect->setEnabled(false);
		ui->btnDisconnect->setEnabled(true);
		ui->fps->show(); ui->toolBar->addWidget(ui->fps);
		ui->statusBar->showMessage("Successfully connected.", 2000);
	}
}

void MainWindow::on_btnDisconnect_clicked()
{
	ui->statusBar->showMessage("Disconnecting...",2000);
	ui->btnDisconnect->setEnabled(false);

	input->disconnect();
	gloveWidget->resetData();
	ui->statusBar->showMessage("Disconnected.", 2000);

	ui->btnConnect->setEnabled(true);
	killTimer(timerID); timerID = 0;
}

#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "GloveWidget.h"
#include "MappingDialog.h"
#include "ColorMap.h"

#include "SerialThread.h"
#include "RandomInput.h"
#if HAVE_ROS
#include "ROSInput.h"
#endif
#include <QCloseEvent>
#include <QComboBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>

#include <iostream>
#include <math.h>
#include <boost/bind.hpp>

using namespace std;

MainWindow::MainWindow(size_t noTaxels, QWidget *parent) :
   QMainWindow(parent), ui(new Ui::MainWindow), iJointIdx(-1),
   input(0), data(noTaxels), display(noTaxels),
   frameCount(-1), timerID(0), gloveWidget(0), mapDlg(0),
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

	connect(ui->actConfMap, SIGNAL(triggered()), this, SLOT(configureMapping()));
	connect(ui->actSaveMapping, SIGNAL(triggered()), this, SLOT(saveMapping()));

	connect(ui->updateTimeSpinBox, SIGNAL(valueChanged(int)), this, SLOT(setTimer(int)));
	connect(ui->lambdaSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setLambda(double)));

	ui->lambdaSpinBox->setValue(0.5);
	data.setRangeLambda(0.9995);
	data.setReleaseDecay(0.1);
	initModeComboBox(ui->modeComboBox);

	// init upper ranges
	for (TactileArray::iterator it=data.begin(), end=data.end(); it!=end; ++it)
		it->init(FLT_MAX, 4095);

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
	cb->setCurrentIndex (TactileSensor::getModeID("default"));
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
	gloveWidget = new GloveWidget(layout, this);
	ui->verticalLayout->insertWidget(0, gloveWidget);
	resetColors();
	gloveWidget->setFocus();

	connect(ui->actSaveSVG, SIGNAL(triggered()), gloveWidget, SLOT(saveSVG()));
	connect(gloveWidget, SIGNAL(doubleClickedNode(uint)), this, SLOT(editMapping(uint)));
	connect(gloveWidget, SIGNAL(unAssignedTaxels(bool)), ui->actConfMap, SLOT(setEnabled(bool)));
	gloveWidget->setShowChannels(ui->actShowChannels->isChecked());
	gloveWidget->setShowNames(ui->actShowIDs->isChecked());
	gloveWidget->setShowAllNames(ui->actShowAllIDs->isChecked());
	connect(ui->actShowChannels, SIGNAL(toggled(bool)), gloveWidget, SLOT(setShowChannels(bool)));
	connect(ui->actShowIDs, SIGNAL(toggled(bool)), gloveWidget, SLOT(setShowNames(bool)));
	connect(ui->actShowAllIDs, SIGNAL(toggled(bool)), gloveWidget, SLOT(setShowAllNames(bool)));

	// initialize TaxelMapping
	for (TaxelMapping::const_iterator it=mapping.begin(), end=mapping.end();
	     it!=end; ++it) {
		if (it->second < 0) continue; // ignore channel indexes

		QString sName = QString::fromStdString(it->first);
		int nodeIdx = gloveWidget->findPathNodeIndex(sName);
		if (nodeIdx < 0)
			cerr << "couldn't find a node named " << it->first << endl;

		gloveWidget->setChannel(nodeIdx, it->second);
		nodeToData[nodeIdx] = it->second;
	}
	bDirtyMapping = false;
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
		fMin=0; fMax=4095;
		colorMap = absColorMap;
		break;

	case TactileSensor::absCurrent:
	case TactileSensor::absMean:
	case TactileSensor::dynCurrent:
	case TactileSensor::dynMean:
		fMin=0; fMax=1;
		colorMap = absColorMap;
		break;

	case TactileSensor::dynCurrentRelease:
	case TactileSensor::dynMeanRelease:
		fMin=-1; fMax=1;
		colorMap = relColorMap;
		break;
	}
}

void MainWindow::timerEvent(QTimerEvent *event)
{
	if (event->timerId() != timerID) return;
	if (frameCount < 0) return; // no data received yet

	QMutexLocker lock(&dataMutex);
	int fps = -1;

	TactileSensor::Mode mode = (TactileSensor::Mode) ui->modeComboBox->currentIndex();
	data.copyValues(display.begin(), mode);

	if (lastUpdate.elapsed() > 1000) { // update framerate every 1s
		fps = roundf (1000. * frameCount / lastUpdate.restart());
		frameCount = 0;
	}
	if (!gloveWidget) return;
	lock.unlock();

	ColorMap *colorMap;
	float fMin, fMax;
	chooseMapping(mode, colorMap, fMin, fMax);

	for (NodeToDataMap::const_iterator it = nodeToData.begin(), end = nodeToData.end();
	     it != end; ++it) {
		if (highlighted.contains(it.key())) continue;
		QColor color = colorMap->map(display[it.value()], fMin, fMax);
		gloveWidget->updateColor(it.key(), color);
	}
	gloveWidget->updateSVG();

	// update MappingDialog if present
	if (mapDlg) {
		std::vector<float> display(data.size());
		mode = TactileSensor::absCurrent;
		chooseMapping(mode, colorMap, fMin, fMax);
		lock.relock();
		data.copyValues(display.begin(), mode);
		lock.unlock();
		mapDlg->update(display, colorMap, fMin, fMax);
	}

	if (iJointIdx >= 0) updateJointBar(display[iJointIdx]);
	if (fps >= 0) ui->fps->setText(QString("%1 fps").arg(fps));
}

void MainWindow::resetColors(const QColor &color) {
	for (NodeToDataMap::const_iterator it = nodeToData.begin(), end = nodeToData.end();
	     it != end; ++it)
		gloveWidget->updateColor(it.key(), color);
	gloveWidget->updateSVG();
}

void MainWindow::updateJointBar(unsigned short value)
{
	const int min=4095;
	const int max=2000;
	const int targetRange=100;
	ui->jointBar->setValue(((value-min) * targetRange) / (max-min));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	if (bDirtyMapping) {
		QMessageBox::StandardButton result
		      = QMessageBox::warning(this, "Unsaved taxel mapping",
		                             "You have unsaved changes to the taxel mapping. Close anyway?",
		                             QMessageBox::Save | QMessageBox::Ok | QMessageBox::Cancel,
		                             QMessageBox::Cancel);
		if (result == QMessageBox::Save) saveMapping();
		if (result == QMessageBox::Cancel) {
			event->ignore();
			return;
		}
	}

	if (gloveWidget && !gloveWidget->canClose())
		event->ignore();
}

/*** functions for taxel mapping configuration ***/
void MainWindow::editMapping(unsigned int nodeIdx)
{
	static const QColor highlightColor("blue");
	highlighted.insert(nodeIdx);
	QString oldStyle=gloveWidget->highlight(nodeIdx, highlightColor);

	bool bOwnDialog = false;
	if (!mapDlg) {
		mapDlg = new MappingDialog(this);
		connect(mapDlg, SIGNAL(destroyed()), this, SLOT(resetMapDlg()));
		bOwnDialog = true;
	}
	const QString &oldName = gloveWidget->getNodeName(nodeIdx);
	const NodeToDataMap::const_iterator node=nodeToData.find(nodeIdx);
	const int channel = node == nodeToData.end() ? -1 : node.value();
	mapDlg->init(oldName, channel, data.size(), getUnassignedChannels());

	int res = mapDlg->exec();
	gloveWidget->restore(nodeIdx, oldStyle);
	highlighted.remove(nodeIdx);

	if (res == QDialog::Accepted) {
		QString newName = mapDlg->name().trimmed();
		if (oldName != mapDlg->name() && !newName.isEmpty()) {
			if (gloveWidget->findPathNodeIndex(newName))
				QMessageBox::warning(this, "Name clash", "Taxel name already in use.", QMessageBox::Ok);
			else { // change node id in gloveWidget's DOM
				gloveWidget->setNodeName(nodeIdx, newName);
				gloveWidget->updateSVG();
			}
		}
		if (mapDlg->channel() != channel) {
			bDirtyMapping = true;
			if (mapDlg->channel() < 0) nodeToData.remove(nodeIdx);
			else nodeToData[nodeIdx] = mapDlg->channel();
			gloveWidget->setChannel(nodeIdx, mapDlg->channel());
		}
	}
	if (bOwnDialog) mapDlg->deleteLater();
}

void MainWindow::resetMapDlg ()
{
	mapDlg = 0;
}

void MainWindow::setCancelConfigure(bool bCancel)
{
	bCancelConfigure = bCancel;
}

void MainWindow::configureMapping()
{
	mapDlg = new MappingDialog(this);
	connect(mapDlg, SIGNAL(destroyed()), this, SLOT(resetMapDlg()));
	QPoint         pos;

	bCancelConfigure = false;
	QPushButton *btn = mapDlg->addButton(tr("&Abort"), QDialogButtonBox::DestructiveRole);
	connect(btn, SIGNAL(clicked()), this, SLOT(setCancelConfigure()));
	connect(btn, SIGNAL(clicked()), mapDlg, SLOT(reject()));

	// display channel numbers, but not the names
	bool bShowChannels = ui->actShowChannels->isChecked();
	bool bShowNames = ui->actShowIDs->isChecked();
	bool bShowAllNames = ui->actShowAllIDs->isChecked();
	ui->actShowAllIDs->setChecked(false);
	ui->actShowChannels->setChecked(true);
	ui->actShowIDs->setChecked(false);

	for (unsigned int nodeIdx = 0, end=gloveWidget->numNodes();
	     !bCancelConfigure && nodeIdx!=end; ++nodeIdx) {
		const QString &oldName = gloveWidget->getNodeName(nodeIdx);
		// ignore nodes named path*
		if (oldName.startsWith("path")) continue;
		// and nodes already assigned
		if (nodeToData.find(nodeIdx) != nodeToData.end()) continue;

		editMapping(nodeIdx);
		pos = mapDlg->pos();
		mapDlg->show();
		mapDlg->move(pos);
	}
	mapDlg->deleteLater();

	// restore channel/name display
	ui->actShowChannels->setChecked(bShowChannels);
	ui->actShowIDs->setChecked(bShowNames);
	ui->actShowAllIDs->setChecked(bShowAllNames);
}

QList<unsigned int> MainWindow::getUnassignedChannels() const
{
	QList<unsigned int> unassigned;
	for (size_t i=0, end=data.size(); i < end; ++i)
		unassigned.append(i);

	for (NodeToDataMap::const_iterator it = nodeToData.begin(), end = nodeToData.end();
	     it != end; ++it)
		unassigned.removeOne(it.value());
	return unassigned;
}

void MainWindow::saveMapping()
{
	typedef std::map<QString, std::pair<QString, void (GloveWidget::*)(QTextStream&)> > FilterMap;
	static FilterMap filterMap;
	static QString sFilters;
	static FilterMap::const_iterator defaultFilter;
	if (filterMap.empty()) {
		filterMap[tr("mapping configs (*.cfg)")] = make_pair(".cfg", &GloveWidget::saveMappingCfg);
		defaultFilter = filterMap.begin();
		filterMap[tr("xacro configs (*.yaml)")] = make_pair(".yaml", &GloveWidget::saveMappingYAML);
		for (FilterMap::const_iterator it=filterMap.begin(), end=filterMap.end();
		     it != end; ++it) {
			if (!sFilters.isEmpty()) sFilters.append(";;");
			sFilters.append(it->first);
		}
	}

	// choose default filter from current file name
	QString selectedFilter = defaultFilter->first;
	for (FilterMap::const_iterator it=filterMap.begin(), end=filterMap.end(); it != end; ++it) {
		if (sMappingFile.endsWith(it->second.first))
			selectedFilter = it->first;
	}

	// exec file dialog
	QString sFileName = QFileDialog::getSaveFileName(0, "save taxel mapping",
	                                                 sMappingFile, sFilters, &selectedFilter);
	if (sFileName.isNull()) return;

	// which filter was choosen?
	FilterMap::const_iterator chosenFilter = filterMap.find(selectedFilter);
	for (FilterMap::const_iterator it=filterMap.begin(), end=filterMap.end(); it != end; ++it) {
		if (sFileName.endsWith(it->second.first))
			chosenFilter = it;
	}
	if (chosenFilter == filterMap.end()) chosenFilter = defaultFilter;

	// append default extension from selected filter
	QFileInfo fi(sFileName);
	if (fi.suffix().isEmpty()) sFileName.append(chosenFilter->second.first);

	// save name of mapping file for next saving attempt
	sMappingFile = sFileName;

	QFile file(sFileName);
	if (!file.open(QFile::WriteOnly | QFile::Truncate)) {
		QMessageBox::warning(this, "save taxel mapping",
		                     QString("Failed to open file for writing:\n%1").arg(sFileName));
		return;
	}

	QTextStream ts(&file);
	(gloveWidget->*chosenFilter->second.second)(ts);
	bDirtyMapping = false;
}

/*** functions for connection handling ***/
void MainWindow::updateData(const InputInterface::data_vector &taxels) {
	QMutexLocker lock(&dataMutex);
	assert(taxels.size() == data.size());

	data.updateValues(taxels.begin(), taxels.end());
	++frameCount;
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
		frameCount = -1; lastUpdate.start();
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
	resetColors(QColor("black"));
	ui->statusBar->showMessage("Disconnected.", 2000);
	resetColors();

	ui->btnConnect->setEnabled(true);
	killTimer(timerID); timerID = 0;
}

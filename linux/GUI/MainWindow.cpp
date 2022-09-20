#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "GloveWidget.h"
#include "MappingDialog.h"
#include "FileExistsDialog.h"
#include "ColorMap.h"

#include "QSerialInput.h"
#include "RandomInput.h"
#if HAVE_ROS
#include "ROSInput.h"
#endif

#include <tactile_filters/PieceWiseLinearCalib.h>
#include <QCloseEvent>
#include <QComboBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>

#include <iostream>
#include <math.h>
#include <qtimer.h>
#include <boost/bind.hpp>

using namespace std;
using tactile::PieceWiseLinearCalib;
using tactile::TactileValue;
using tactile::TactileValueArray;

MainWindow::MainWindow(size_t noTaxels, QWidget *parent)
  : QMainWindow(parent)
  , ui(new Ui::MainWindow)
  , gloveWidget(nullptr)
  , input(nullptr)
  , data(noTaxels)
  , display(noTaxels)
  , mapDlg(nullptr)
  , iJointIdx(-1)
  , absColorMap(nullptr)
  , relColorMap(nullptr)
  , frameCount(-1)
  , timerID(0)
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
	for (auto &value : data)
		value.init(FLT_MAX, 4095);

	// init color maps
	QStringList colorNames;
	absColorMap = new ColorMap;
	colorNames << "black"
	           << "lime"
	           << "yellow"
	           << "red";
	absColorMap->append(colorNames);

	relColorMap = new ColorMap;
	colorNames.clear();
	colorNames << "red"
	           << "black"
	           << "lime";
	relColorMap->append(colorNames);
}

void MainWindow::initModeComboBox(QComboBox *cb)
{
	QStringList items;
	for (unsigned int m = 0, e = TactileValue::lastMode; m <= e; ++m)
		items << TactileValue::getModeName((TactileValue::Mode)m).c_str();
	cb->addItems(items);
	cb->setCurrentIndex(TactileValue::getMode("default"));
}

void MainWindow::setCalibration(const std::string &sCalibFile)
{
	QMutexLocker lock(&dataMutex);
	calib.reset(new PieceWiseLinearCalib(PieceWiseLinearCalib::load(sCalibFile)));
	for (unsigned int id : nodeToData) {
		data[id].setCalibration(calib);
	}

	// init upper ranges
	float fMax = calib->output_range().max();
	for (auto &value : data)
		value.init(FLT_MAX, fMax);
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

void MainWindow::initJointBar(TaxelMapping &mapping)
{
	// do we have a joint value?
	TaxelMapping::iterator it = mapping.find("bar");
	if (it != mapping.end()) {
		iJointIdx = it->second;
		mapping.erase(it);
	} else
		iJointIdx = -1;

	if (iJointIdx < 0 || static_cast<size_t>(iJointIdx) >= data.size()) {
		iJointIdx = -1;
		ui->jointBar->hide();
	} else
		ui->jointBar->show();
}

void MainWindow::initGloveWidget(const QString &layout, bool bMirror, const TaxelMapping &mapping)
{
	// do before creating the GloveWidget to allow for removing of the bar mapping
	initJointBar(const_cast<TaxelMapping &>(mapping));

	QMutexLocker lock(&dataMutex);
	if (gloveWidget) {
		on_btnDisconnect_clicked();
		ui->verticalLayout->removeWidget(gloveWidget);
		delete gloveWidget;
		gloveWidget = nullptr;
	}
	gloveWidget = new GloveWidget(layout, bMirror, this);
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
	for (const auto &item : mapping) {
		QString sName = QString::fromStdString(item.first);
		int nodeIdx = gloveWidget->findPathNodeIndex(sName);
		if (nodeIdx < 0)
			cerr << "couldn't find a node named " << item.first << endl;

		gloveWidget->setChannel(nodeIdx, item.second);
		nodeToData[nodeIdx] = item.second;
	}
	bDirtyMapping = false;
}

void MainWindow::setTimer(int interval)
{
	if (!timerID)
		return;
	killTimer(timerID);
	timerID = startTimer(interval);
}

void MainWindow::setLambda(double value)
{
	data.setMeanLambda(value);
}

void MainWindow::chooseMapping(TactileValue::Mode mode, const std::shared_ptr<tactile::Calibration> &calib,
                               ColorMap *&colorMap, float &fMin, float &fMax)
{
	switch (mode) {
	case TactileValue::rawCurrent:
	case TactileValue::rawMean:
		if (calib) {
			tactile::Range r = calib->output_range();
			fMin = r.min();
			fMax = r.max();
		} else {
			fMin = 0;
			fMax = 4095;
		}
		colorMap = absColorMap;
		break;

	case TactileValue::absCurrent:
	case TactileValue::absMean:
	case TactileValue::dynCurrent:
	case TactileValue::dynMean:
		fMin = 0;
		fMax = 1;
		colorMap = absColorMap;
		break;

	case TactileValue::dynCurrentRelease:
	case TactileValue::dynMeanRelease:
		fMin = -1;
		fMax = 1;
		colorMap = relColorMap;
		break;
	}
}

void MainWindow::timerEvent(QTimerEvent *event)
{
	if (event->timerId() != timerID)
		return;
	if (frameCount < 0)
		return;  // no data received yet

	QMutexLocker lock(&dataMutex);
	int fps = -1;

	TactileValue::Mode mode = (TactileValue::Mode)ui->modeComboBox->currentIndex();
	data.getValues(mode, display);

	if (lastUpdate.elapsed() > 1000) {  // update framerate every 1s
		fps = roundf(1000. * frameCount / lastUpdate.restart());
		frameCount = 0;
	}
	if (!gloveWidget)
		return;
	lock.unlock();

	ColorMap *colorMap;
	float fMin, fMax;
	chooseMapping(mode, calib, colorMap, fMin, fMax);

	for (NodeToDataMap::const_iterator it = nodeToData.begin(), end = nodeToData.end(); it != end; ++it) {
		if (highlighted.contains(it.key()))
			continue;
		QColor color = colorMap->map(display[it.value()], fMin, fMax);
		gloveWidget->updateColor(it.key(), color);
	}
	gloveWidget->updateSVG();

	// update MappingDialog if present
	if (mapDlg) {
		std::vector<float> display(data.size());
		mode = TactileValue::absCurrent;
		chooseMapping(mode, calib, colorMap, fMin, fMax);
		lock.relock();
		data.getValues(mode, display);
		lock.unlock();
		mapDlg->update(display, colorMap, fMin, fMax);
	}

	if (iJointIdx >= 0)
		updateJointBar(data[iJointIdx].value(TactileValue::rawCurrent));
	if (fps >= 0)
		ui->fps->setText(QString("%1 fps").arg(fps));
}

void MainWindow::resetColors(const QColor &color)
{
	for (NodeToDataMap::const_iterator it = nodeToData.begin(), end = nodeToData.end(); it != end; ++it)
		gloveWidget->updateColor(it.key(), color);
	gloveWidget->updateSVG();
}

void MainWindow::updateJointBar(unsigned short value)
{
	const int min = 4095;
	const int max = 2000;
	const int targetRange = 100;
	ui->jointBar->setValue(((value - min) * targetRange) / (max - min));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	if (bDirtyMapping) {
		QMessageBox::StandardButton result = QMessageBox::warning(
		    this, "Unsaved taxel mapping", "You have unsaved changes to the taxel mapping. Close anyway?",
		    QMessageBox::Save | QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
		if (result == QMessageBox::Save)
			saveMapping();
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
	QString oldStyle = gloveWidget->highlight(nodeIdx, highlightColor);

	bool bOwnDialog = false;
	if (!mapDlg) {
		mapDlg = new MappingDialog(this);
		connect(mapDlg, SIGNAL(destroyed()), this, SLOT(resetMapDlg()));
		bOwnDialog = true;
	}
	const QString &oldName = gloveWidget->getNodeName(nodeIdx);
	const NodeToDataMap::const_iterator node = nodeToData.find(nodeIdx);
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
			else {  // change node id in gloveWidget's DOM
				gloveWidget->setNodeName(nodeIdx, newName);
				gloveWidget->updateSVG();
			}
		}
		if (mapDlg->channel() != channel) {
			bDirtyMapping = true;
			if (mapDlg->channel() < 0)
				nodeToData.remove(nodeIdx);
			else
				nodeToData[nodeIdx] = mapDlg->channel();
			gloveWidget->setChannel(nodeIdx, mapDlg->channel());
		}
	}
	if (bOwnDialog)
		mapDlg->deleteLater();
}

void MainWindow::resetMapDlg()
{
	mapDlg = nullptr;
}

void MainWindow::setCancelConfigure(bool bCancel)
{
	bCancelConfigure = bCancel;
}

void MainWindow::configureMapping()
{
	mapDlg = new MappingDialog(this);
	connect(mapDlg, SIGNAL(destroyed()), this, SLOT(resetMapDlg()));
	QPoint pos;

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

	for (unsigned int nodeIdx = 0, end = gloveWidget->numNodes(); !bCancelConfigure && nodeIdx != end; ++nodeIdx) {
		const QString &oldName = gloveWidget->getNodeName(nodeIdx);
		// ignore nodes named path*
		if (oldName.startsWith("path"))
			continue;
		// and nodes already assigned
		if (nodeToData.find(nodeIdx) != nodeToData.end())
			continue;

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
	for (size_t i = 0, end = data.size(); i < end; ++i)
		unassigned.append(i);

	for (NodeToDataMap::const_iterator it = nodeToData.begin(), end = nodeToData.end(); it != end; ++it)
		unassigned.removeOne(it.value());
	return unassigned;
}

void MainWindow::saveMapping()
{
	using FilterMap = std::map<QString, std::pair<QString, bool (GloveWidget::*)(const QString &, const QString &)> >;
	static FilterMap filterMap;
	static QString sFilters;
	static FilterMap::const_iterator defaultFilter;
	if (filterMap.empty()) {
		filterMap[tr("mapping configs (*.cfg)")] = make_pair(".cfg", &GloveWidget::saveMappingCfg);
		defaultFilter = filterMap.begin();
		filterMap[tr("xacro configs (*.yaml)")] = make_pair(".yaml", &GloveWidget::saveMappingYAML);
		for (const auto &item : filterMap) {
			if (!sFilters.isEmpty())
				sFilters.append(";;");
			sFilters.append(item.first);
		}
	}

	// choose default filter from current file name
	QString selectedFilter = defaultFilter->first;
	for (const auto &item : filterMap) {
		if (sMappingFile.endsWith(item.second.first))
			selectedFilter = item.first;
	}

	// exec file dialog
	QFileDialog dlg(this, "save taxel mapping", sMappingFile, sFilters);
	dlg.setFileMode(QFileDialog::AnyFile);
	dlg.selectNameFilter(selectedFilter);
	dlg.selectFile(sMappingFile);
	dlg.setLabelText(QFileDialog::Accept, tr("Save"));
	QString sFileName;
	QString mapping_name;
	FilterMap::const_iterator chosenFilter = filterMap.end();
	while (true) {
		if (dlg.exec() == QDialog::Rejected)
			return;

		selectedFilter = dlg.selectedNameFilter();
		sFileName = dlg.selectedFiles().first();

		// which filter was choosen?
		chosenFilter = filterMap.find(selectedFilter);
		// file suffix superseeds chosen filter
		for (FilterMap::const_iterator it = filterMap.begin(), end = filterMap.end(); it != end; ++it) {
			if (sFileName.endsWith(it->second.first))
				chosenFilter = it;
		}
		if (chosenFilter == filterMap.end())
			chosenFilter = defaultFilter;

		// append default extension from selected filter
		QFileInfo fi(sFileName);
		if (fi.suffix().isEmpty())
			sFileName.append(chosenFilter->second.first);

		if (fi.exists()) {
			FileExistsDialog msg(sFileName, chosenFilter->second.second == &GloveWidget::saveMappingCfg, this);
			int result = msg.exec();
			if (result == QDialog::Rejected)
				continue;
			if (result == QDialogButtonBox::YesRole)
				QFile(sFileName).remove();
			mapping_name = msg.mappingName();
		}
		break;
	}

	// save name of mapping file for next saving attempt
	sMappingFile = sFileName;

	if ((gloveWidget->*chosenFilter->second.second)(sFileName, mapping_name))
		bDirtyMapping = false;
	else
		QMessageBox::warning(this, "Save taxel mapping", QString("Failed to open file for writing:\n%1").arg(sFileName));
}

/*** functions for connection handling ***/
template <typename value_type>
void MainWindow::updateData(const std::vector<value_type> &frame)
{
	QMutexLocker lock(&dataMutex);
	assert(frame.size() == data.size());
	data.updateValues(frame.begin(), frame.end());
	++frameCount;
}

void MainWindow::configSerial(const QString &sDevice)
{
	ui->inputLineEdit->setText(sDevice);
	ui->inputLineEdit->setToolTip("serial device name");

	QSerialInput *serial = new QSerialInput(data.size());
	serial->setUpdateFunction(boost::bind(&MainWindow::updateData<tactile::InputInterface::data_type>, this, _1));
	connect(serial, SIGNAL(statusMessage(QString, int)), ui->statusBar, SLOT(showMessage(QString, int)));
	connect(serial, SIGNAL(disconnected(QString)), this, SLOT(onSerialError(QString)));
	input = serial;
}
void MainWindow::onSerialError(const QString &reason)
{
	if (!reason.isEmpty()) {
		QTimer::singleShot(50, this, SLOT(on_btnDisconnect_clicked()));
		statusBar()->showMessage(reason, 5000);
	}
}

void MainWindow::configROS(const QString &sTopic)
{
#if HAVE_ROS
	ui->inputLineEdit->setText(sTopic);
	ui->inputLineEdit->setToolTip("ROS topic");

	ROSInput *rosInput = new ROSInput(data.size());
	rosInput->setUpdateFunction(boost::bind(&MainWindow::updateData<float>, this, _1));
	connect(rosInput, SIGNAL(statusMessage(QString, int)), ui->statusBar, SLOT(showMessage(QString, int)));
	input = rosInput;

	on_btnConnect_clicked();  // auto-connect to ROS topic
#endif
}

void MainWindow::configRandom()
{
	ui->verticalLayout->addWidget(ui->inputLineEdit);
	ui->inputLineEdit->hide();
	RandomInput *random = new RandomInput(data.size());
	random->setUpdateFunction(boost::bind(&MainWindow::updateData<tactile::InputInterface::data_type>, this, _1));
	input = random;
}

void MainWindow::on_btnConnect_clicked()
{
	if (input->connect(ui->inputLineEdit->text())) {
		ui->statusBar->showMessage("Successfully connected.", 2000);
		frameCount = -1;
		lastUpdate.start();
		timerID = startTimer(ui->updateTimeSpinBox->value());

		ui->btnConnect->setEnabled(false);
		ui->btnDisconnect->setEnabled(true);
		ui->fps->show();
		ui->toolBar->addWidget(ui->fps);
	}
}

void MainWindow::on_btnDisconnect_clicked()
{
	ui->btnDisconnect->setEnabled(false);

	input->disconnect();
	if (ui->statusBar->currentMessage().isEmpty())
		ui->statusBar->showMessage("Disconnected.", 2000);
	resetColors();

	ui->btnConnect->setEnabled(true);
	killTimer(timerID);
	timerID = 0;
}

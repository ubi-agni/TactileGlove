#pragma once
#include "ui_MappingDialog.h"

class MappingDialog : public QDialog, private Ui::MappingDialog
{
	Q_OBJECT
	Q_PROPERTY(int channel READ channel WRITE setChannel)

public:
	explicit MappingDialog(const QString &sName, int channel, int maxChannel, QWidget *parent = 0);

	QString name() const;
	int channel() const;

public slots:
	void setChannel(int channel);
};

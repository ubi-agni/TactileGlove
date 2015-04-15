#pragma once
#include "ui_MappingDialog.h"

class MappingDialog : public QDialog, private Ui::MappingDialog
{
	Q_OBJECT

public:
	explicit MappingDialog(const QString &sName, int channel, QWidget *parent = 0);

	QString name() const;
	int channel() const;
};

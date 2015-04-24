#pragma once
#include "ui_MappingDialog.h"

class MappingDialog : public QDialog, private Ui::MappingDialog
{
	Q_OBJECT
	Q_PROPERTY(int channel READ channel WRITE setChannel)

public:
	explicit MappingDialog(QWidget *parent = 0);
	void init(const QString &sName, int channel, int maxChannel);

	QString name() const;
	int channel() const;
	QPushButton* addButton(const QString& label, QDialogButtonBox::ButtonRole role);

public slots:
	void setChannel(int channel);
};

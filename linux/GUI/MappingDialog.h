#pragma once
#include "ui_MappingDialog.h"
#include <QIntValidator>

class ChannelValidator;
class TaxelSelector;
class MappingDialog : public QDialog, private Ui::MappingDialog
{
	Q_OBJECT
	Q_PROPERTY(int channel READ channel WRITE setChannel)

public:
	explicit MappingDialog(QWidget *parent = 0);
	void init(const QString &sName, int channel, int maxChannel, QList<unsigned int> unAssignedChannels);

	QString name() const;
	int channel() const;
	QPushButton* addButton(const QString& label, QDialogButtonBox::ButtonRole role);

public slots:
	void setChannel(int channel);

private:
	ChannelValidator *validator;
	TaxelSelector    *taxelSelector;
};

class ChannelValidator : public QIntValidator {
public:
	ChannelValidator(QObject *parent = 0) : QIntValidator(1, 1, parent) {}
	QValidator::State	validate(QString &input, int &pos) const;
};

#include "MappingDialog.h"
#include "GloveWidget.h"
#include <QPushButton>

MappingDialog::MappingDialog(QWidget *parent) :
   QDialog(parent)
{
	setupUi(this);
}

void MappingDialog::init(const QString &sName, int channel, int maxChannel)
{
	nameEdit->setText(sName);
	channelSpinBox->setMaximum(maxChannel);
	setChannel(channel);
	channelSpinBox->setFocus();
}

QString MappingDialog::name() const {return nameEdit->text();}

int MappingDialog::channel() const {return channelSpinBox->value()-1;}

QPushButton *MappingDialog::addButton(const QString &label, QDialogButtonBox::ButtonRole role)
{
	QPushButton* b = new QPushButton(label);
	buttonBox->addButton(b, role);
	return b;
}

void MappingDialog::setChannel(int channel)
{
	channelSpinBox->setValue(channel+1);
}

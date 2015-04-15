#include "MappingDialog.h"
#include "GloveWidget.h"

MappingDialog::MappingDialog(const QString &sName, int channel, QWidget *parent) :
   QDialog(parent)
{
	setupUi(this);
	nameEdit->setText(sName);
	channelSpinBox->setMaximum(NO_TAXELS);
	channelSpinBox->setValue(channel+1);
	channelSpinBox->setFocus();
}

QString MappingDialog::name() const {return nameEdit->text();}

int MappingDialog::channel() const {return channelSpinBox->value()-1;}

#include "MappingDialog.h"
#include "GloveWidget.h"
#include "TaxelSelector.h"

#include <QPushButton>

static QString unUsed("unused");

QValidator::State ChannelValidator::validate(QString &input, int &pos) const
{
	if (input.isEmpty() || input == unUsed)
		return Acceptable;
	if (unUsed.startsWith(input)) {
		input = unUsed;
		return Intermediate;
	}
	return QIntValidator::validate(input, pos);
}

MappingDialog::MappingDialog(QWidget *parent) : QDialog(parent), taxelSelector(nullptr)
{
	setupUi(this);
	validator = new ChannelValidator(this);
	channelComboBox->setValidator(validator);
}

void MappingDialog::init(const QString &sName, int channel, int maxChannel,
                         const QList<unsigned int> &unAssignedChannels)
{
	validator->setTop(maxChannel);
	nameEdit->setText(sName);
	channelComboBox->clear();
	channelComboBox->addItem(unUsed);
	for (unsigned int ch : unAssignedChannels) {
		channelComboBox->addItem(QString::number(ch + 1));
	}
	setChannel(channel);
	channelComboBox->setFocus();

	if (!unAssignedChannels.empty()) {
		if (!taxelSelector) {
			taxelSelector = new TaxelSelector(this);
			connect(taxelSelector, SIGNAL(selectedChannel(int)), this, SLOT(setChannel(int)));
			connect(taxelSelector, SIGNAL(doubleClicked()), this, SLOT(accept()));
		}
		taxelSelector->init(unAssignedChannels, channel < 0);
		verticalLayout->addWidget(taxelSelector);
	} else if (taxelSelector) {
		delete taxelSelector;
		taxelSelector = nullptr;
	}
}

QString MappingDialog::name() const
{
	return nameEdit->text();
}

int MappingDialog::channel() const
{
	if (channelComboBox->currentText().isEmpty() || channelComboBox->currentText() == unUsed)
		return -1;
	return channelComboBox->currentText().toInt() - 1;
}

void MappingDialog::setChannel(int channel)
{
	if (channel < 0)
		channelComboBox->setCurrentText(unUsed);
	else
		channelComboBox->setCurrentText(QString::number(channel + 1));
}

QPushButton *MappingDialog::addButton(const QString &label, QDialogButtonBox::ButtonRole role)
{
	QPushButton *b = new QPushButton(label);
	buttonBox->addButton(b, role);
	return b;
}

void MappingDialog::update(const std::vector<float> &data, const ColorMap *colorMap, float fMin, float fMax)
{
	if (taxelSelector)
		taxelSelector->update(data, colorMap, fMin, fMax);
}

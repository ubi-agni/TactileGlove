#include "MySpinBox.h"

MySpinBox::MySpinBox(QWidget *parent) :
   QSpinBox(parent)
{
}

int MySpinBox::valueFromText(const QString &input) const
{
	if (input.isEmpty()) return 0;
	return QSpinBox::valueFromText(input);
}


QValidator::State MySpinBox::validate(QString &input, int &pos) const
{
	if (input.isEmpty()) return QValidator::Acceptable;
	return QSpinBox::validate(input, pos);
}

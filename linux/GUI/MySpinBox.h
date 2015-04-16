#pragma once
#include <QSpinBox>

class MySpinBox : public QSpinBox
{
	Q_OBJECT
public:
	explicit MySpinBox(QWidget *parent = 0);
	int valueFromText(const QString &text) const;
	QValidator::State validate(QString &input, int &pos) const;
};

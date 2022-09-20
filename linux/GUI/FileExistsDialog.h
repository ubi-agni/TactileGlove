#pragma once
#include <QDialog>

class QPushButton;
class Ui_FileExistsDialog;
class FileExistsDialog : public QDialog
{
	Q_OBJECT

public:
	explicit FileExistsDialog(const QString &file, bool allow_merge, QWidget *parent = nullptr);
	~FileExistsDialog() override;

	QString mappingName() const;

private Q_SLOTS:
	void _done();
	void _onNameChanged();

private:
	Ui_FileExistsDialog *ui;
	QPushButton *button_merge;
};

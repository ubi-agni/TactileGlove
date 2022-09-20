#include "FileExistsDialog.h"
#include "ui_FileExistsDialog.h"

#include <QPushButton>

FileExistsDialog::FileExistsDialog(const QString &file, bool allow_merge, QWidget *parent)
  : QDialog(parent), ui(new Ui::FileExistsDialog), button_merge(nullptr)
{
	ui->setupUi(this);
	ui->label_exists->setText(tr("File %1 already exists.").arg(file));
	ui->edit_name->setVisible(allow_merge);

	QAbstractButton *button;
	button = ui->button_box->addButton(tr("Overwrite"), QDialogButtonBox::YesRole);
	connect(button, SIGNAL(clicked()), this, SLOT(_done()));
	if (allow_merge) {
		button = button_merge = ui->button_box->addButton(tr("Merge"), QDialogButtonBox::NoRole);
		connect(button, SIGNAL(clicked()), this, SLOT(_done()));
		connect(ui->edit_name, SIGNAL(textChanged(QString)), this, SLOT(_onNameChanged()));
		_onNameChanged();
	}
}

FileExistsDialog::~FileExistsDialog()
{
	delete ui;
}

QString FileExistsDialog::mappingName() const
{
	return ui->edit_name->text().trimmed();
}

void FileExistsDialog::_done()
{
	done(ui->button_box->buttonRole(static_cast<QAbstractButton *>(sender())));
}

void FileExistsDialog::_onNameChanged()
{
	button_merge->setDisabled(mappingName().isEmpty());
}

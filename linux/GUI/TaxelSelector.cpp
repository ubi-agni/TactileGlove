/* ============================================================
 *
 * Copyright (C) 2015 by Robert Haschke <rhaschke at techfak dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the "LGPL"),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CITEC, "Cognitive Interaction Technology" Excellence Cluster
 *     Bielefeld University
 *
 * ============================================================ */

#include "TaxelSelector.h"
#include "ColorMap.h"
#include <QPainter>
#include <QMouseEvent>

TaxelSelector::TaxelSelector(QWidget *parent) :
   QWidget(parent)
{
}

void TaxelSelector::init(const QList<unsigned int> &unassigned)
{
	this->unassigned = unassigned;
	this->colors.clear();
	QColor c("black");
	for (unsigned int i=0; i < unassigned.size(); ++i)
		this->colors.push_back(c);

	QFont font;
	QFontMetrics fm(font);
	cellWidth = ((fm.width("999") + 9) / 10) * 10;
}

void TaxelSelector::update(const std::vector<float> &data,
                                 const ColorMap *colorMap, float fMin, float fMax)
{
	for (unsigned int i=0; i < unassigned.size(); ++i) {
		colors[i] = colorMap->map(data[unassigned[i]], fMin, fMax);
	}
	QWidget::update();
}

QSize TaxelSelector::minimumSizeHint() const
{
	int cols = 10, rows = (unassigned.size() + cols-1) / cols;
	return QSize(cols*cellWidth, rows*cellWidth);
}

void TaxelSelector::paintEvent(QPaintEvent *)
{
	int cols = width() / cellWidth;

	QPainter painter(this);
	painter.setBrush(this->palette().color(QPalette::Window));
	painter.setPen(this->palette().color(QPalette::Window));
	painter.drawRect(QRectF(0,0, width(),height()));
	QRect bounds (0,0, cellWidth, cellWidth);

	for (int r=0, c=0, idx = 0; idx != unassigned.size(); ++idx) {
		bounds.moveTopLeft(QPoint(c*cellWidth, r*cellWidth));
		painter.setBrush(colors[idx]);
		painter.setPen(Qt::white);
		painter.drawRect(bounds);

		QString label = QString::number(unassigned[idx]+1);
		painter.setPen(Qt::black);
		painter.drawText(bounds, Qt::AlignCenter, label);
		painter.setPen(Qt::white);
		painter.drawText(bounds.translated(1,1), Qt::AlignCenter, label);

		if (++c >= cols) {++r; c=0;}
	}
}

void TaxelSelector::mousePressEvent(QMouseEvent *event)
{
	if (event->button() != Qt::LeftButton) return;
	int cols = width() / cellWidth;

	const QPoint &pos = event->pos();
	int idx = pos.y() / cellWidth * cols + pos.x() / cellWidth;

	if (idx >= 0 && idx < unassigned.size())
		emit selectedChannel(unassigned[idx]);
	else
		emit selectedChannel(-1);
}

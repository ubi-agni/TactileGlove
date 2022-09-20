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
#pragma once
#include <QWidget>
#include <QColor>

class ColorMap;
class TaxelSelector : public QWidget
{
	Q_OBJECT
public:
	explicit TaxelSelector(QWidget *parent = nullptr);
	void init(const QList<unsigned int> &unassigned, bool bMonitor);
	void update(const std::vector<float> &data, const ColorMap *colorMap, float fMin, float fMax);

signals:
	void selectedChannel(int channel);
	void doubleClicked();

private:
	void doMonitor(const std::vector<float> &data);

	QSize minimumSizeHint() const override;
	void paintEvent(QPaintEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;
	void mouseDoubleClickEvent(QMouseEvent *event) override;

private:
	QList<unsigned int> unassigned;
	QList<QColor> colors;
	int cellWidth;

	QList<float> accumulated;
	bool bMonitor;
};

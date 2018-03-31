/*
 * MapGraphicsItem.cpp
 *
 *  Created on: Feb 23, 2014
 *      Author: xaero
 */

#include <mrm_control_panel/MapGraphicsItem.h>
#include <mrm_control_panel/RobotGraphicsItem.h>

namespace mrm_control_panel {

MapGraphicsItem::MapGraphicsItem(const QPixmap &pixmap,
								 const QPointF& origin,
								 QGraphicsItem* parent,
								 QGraphicsScene *scene)
	: QGraphicsPixmapItem(pixmap, parent) {
	float resolution = Parameters::getInstance()->mapResolution();

	// ROS_ERROR("origin.x() = %f, origin.y() = %f",origin.x(), origin.y());
	setTransform(QTransform::fromTranslate(origin.x(), origin.y()), true);
	setTransform(QTransform::fromScale(resolution, resolution), true);
	// ROS_ERROR("resolution = %f",resolution);
	setTransform(QTransform::fromScale(-1,1), true);
	setTransform(QTransform().rotate(90), true);
}

MapGraphicsItem::~MapGraphicsItem() {
}

void MapGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent* event) {
	emit mousePressed(mapToScene(event->pos()) , event->button());
}

} /* namespace mrm_control_panel */

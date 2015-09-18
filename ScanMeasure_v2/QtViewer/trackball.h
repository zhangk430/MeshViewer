#ifndef TRACKBALL_H
#define TRACKBALL_H

#include <QtGui/qvector3d.h>
#include <QtGui/qquaternion.h>
#include <QtGui/QMatrix4x4>

class TrackBall
{
public:
	TrackBall();
	void push(const QPointF& p);
	void move(const QPointF& p);
	void release(const QPointF& p);
	QQuaternion rotation() const;

	void reset();

private:
	QQuaternion m_rotation;

	QPointF m_lastPos;
	bool m_pressed;
};

#endif
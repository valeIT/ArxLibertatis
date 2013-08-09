/*
 * Copyright 2011-2012 Arx Libertatis Team (see the AUTHORS file)
 *
 * This file is part of Arx Libertatis.
 *
 * Arx Libertatis is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Arx Libertatis is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Arx Libertatis.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ARX_GRAPHICS_EFFECTS_TRAIL_H
#define ARX_GRAPHICS_EFFECTS_TRAIL_H

#include <vector>
#include <boost/circular_buffer.hpp>

#include "math/Vector3.h"
#include "graphics/Color.h"

class Trail {

public:
	Trail(Vec3f & initialPosition);

	void SetNextPosition(Vec3f & nextPosition);

	void Update();
	void Render();

private:
	struct TrailSegment {

		TrailSegment(Color color, float size)
			: m_color(color)
			, m_size(size)
		{}

		Color m_color;
		float m_size;
	};

	std::vector<TrailSegment> m_segments;

	Vec3f m_nextPosition;
	boost::circular_buffer<Vec3f> m_positions;
};

#endif // ARX_GRAPHICS_EFFECTS_TRAIL_H

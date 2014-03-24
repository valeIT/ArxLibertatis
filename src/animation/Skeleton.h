/*
 * Copyright 2011-2014 Arx Libertatis Team (see the AUTHORS file)
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

#ifndef ARX_ANIMATION_SKELETON_H
#define ARX_ANIMATION_SKELETON_H

#include <string>
#include <vector>

#include "glm/gtc/quaternion.hpp"

#include "math/Types.h"

struct VertexGroup {
	std::string       name;
	long              origin;
	std::vector<long> indexes;
	float             siz;
	
	VertexGroup()
		: name()
		, origin(0)
		, indexes()
		, siz(0.0f)
	{}
};

struct BoneTransform {
	glm::quat quat;
	Vec3f     trans;
	Vec3f     scale;
};

struct Bone {
	long              nb_idxvertices;
	long            * idxvertices;
	VertexGroup     * original_group;
	long              father;
	
	BoneTransform     anim;
	BoneTransform     last;
	BoneTransform     init;
	
	Vec3f             transinit_global;
};

struct Skeleton {
	Bone * bones;
	long   nb_bones;
};

#endif // ARX_ANIMATION_SKELETON_H

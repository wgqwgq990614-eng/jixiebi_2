#include "kinematics.h"


static float theta2rad(float theta)
{
	return theta * PI / 180.0f;
}

static float rad2theta(float rad)
{
	return rad * 180.0f / PI;
}

void kinematics_init(KinematicsObjectTypeDef* self)
{
	memset(self, 0, sizeof(KinematicsObjectTypeDef));
}

/*
 * -90 <= knot[0].theta <= 90
 * 0 <= knot[1].theta <= 180
 * -90 <= knot[2].theta <= 90
 * -90 <= knot[3].theta <= 90 
 *
 */
uint8_t ikine(KinematicsObjectTypeDef* self)
{
	float a, b, c, d, length, cos_knot2, sin_knot2;
	KnotObjectTypeDef knots[4];
	
	length = sqrtf(self->vector.x * self->vector.x +
                   self->vector.y * self->vector.y);
	a = length - (LINKAGE_4 * cos(theta2rad(self->alpha)));
	b = self->vector.z - LINKAGE_1 - (LINKAGE_4 * sin(theta2rad(self->alpha)));
	
    knots[0].rad   = atan2f(self->vector.y, self->vector.x);
    knots[0].theta = rad2theta(knots[0].rad);
	
	cos_knot2 = ((a * a) + (b * b) - (LINKAGE_2 * LINKAGE_2) - (LINKAGE_3 * LINKAGE_3)) / (2 * LINKAGE_2 * LINKAGE_3);
	if(cos_knot2 > 1.0f)
	{
		return INVAILD;
	}
	sin_knot2 = -sqrt(1 - (cos_knot2 * cos_knot2));
	knots[2].rad = atan2(sin_knot2, cos_knot2);
	knots[2].theta = rad2theta(knots[2].rad);

	c = LINKAGE_2 + LINKAGE_3 * cos(knots[2].rad);
	d = LINKAGE_3 * sin(knots[2].rad);
	knots[1].rad = atan2(c, d) - atan2(a, b);
	knots[1].theta = rad2theta(knots[1].rad);
	
	knots[3].theta = self->alpha - knots[1].theta - knots[2].theta;
	knots[3].rad = theta2rad(knots[3].theta);
	
	if ((knots[0].theta > MIN_KNOT6_ANGLE) && (knots[0].theta < MAX_KNOT6_ANGLE) &&	\
		(knots[1].theta > MIN_KNOT5_ANGLE) && (knots[1].theta < MAX_KNOT5_ANGLE) &&	\
		(knots[2].theta > MIN_KNOT4_ANGLE) && (knots[2].theta < MAX_KNOT4_ANGLE) &&	\
		(knots[3].theta > MIN_KNOT3_ANGLE) && (knots[3].theta < MAX_KNOT3_ANGLE))
	{
		for(uint8_t i = 0; i < 4; i++)
		{
			self->knot[i].rad = knots[i].rad;
			self->knot[i].theta = knots[i].theta;
		}
		return OK;
	}
	else
	{
		return INVAILD;
	}
}

bool set_z_range(KinematicsObjectTypeDef* self,
                 VectorObjectTypeDef* vector,
                 float pitch,      
                 float z1   
                 )  
{
	float inc;

	self->vector.x = vector->x;
	self->vector.y = vector->y;
	self->alpha = pitch;
	inc = (vector->z > z1) ? -0.1f : 0.1f;
	
	for (self->vector.z =vector->z; (inc > 0) ? (self->vector.z < z1) : (self->vector.z>= z1); self->vector.z+= inc)
	{
		if (ikine(self) == OK)
		{
			return true;
		}
	}	

	return false;
}


bool set_pitch_range(KinematicsObjectTypeDef* self, VectorObjectTypeDef* vector, float alpha1, float alpha2)
{
	float inc;

	self->vector.x = vector->x;
	self->vector.y = vector->y;
	self->vector.z = vector->z;
	inc = (alpha1 > alpha2) ? -1.0f : 1.0f;
	
	for (self->alpha = alpha1; (inc > 0) ? (self->alpha < alpha2) : (self->alpha >= alpha2); self->alpha += inc)
	{
		if (ikine(self) == OK)
		{
			return true;
		}
	}	

	return false;
}


/******************************************************************************
* ISA Flight Software
 * @file vector3.c
 * @brief 3D Vector Operations Implementation
* @details Implements basic 3D vector mathematical operations
* @author Ananthu Dev, Project Engineer, Spacelabs
 * @date 2025
* @version 4.0
*
* MISRA C: Compliant Implementation
*****************************************************************************/

#include "vector3.h"
#include <math.h>

void v3_add(Vector3* r,const Vector3* a,const Vector3* b){r->x=a->x+b->x;r->y=a->y+b->y;r->z=a->z+b->z;}
void v3_sub(Vector3* r,const Vector3* a,const Vector3* b){r->x=a->x-b->x;r->y=a->y-b->y;r->z=a->z-b->z;}
void v3_cross(Vector3* r,const Vector3* a,const Vector3* b){r->x=a->y*b->z-a->z*b->y; r->y=a->z*b->x-a->x*b->z; r->z=a->x*b->y-a->y*b->x;}
double v3_dot(const Vector3* a,const Vector3* b){return a->x*b->x+a->y*b->y+a->z*b->z;}
void v3_scale(Vector3* r,const Vector3* a,double s){r->x=a->x*s; r->y=a->y*s; r->z=a->z*s;}
double v3_norm(const Vector3* a){return sqrt(v3_dot(a,a));}
void v3_normed(Vector3* r,const Vector3* a){double n=v3_norm(a); if(n==0){r->x=r->y=r->z=0;} else v3_scale(r,a,1.0/n);}


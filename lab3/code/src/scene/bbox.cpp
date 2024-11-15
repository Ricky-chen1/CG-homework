#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL
{

  bool BBox::intersect(const Ray &r, double &t0, double &t1) const
  {

    // TODO (Part 2.2):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bouding box within the range given by
    // t0, t1, update t0 and t1 with the new intersection times.
    double tx_min = std::min((min.x - r.o.x) / r.d.x, (max.x - r.o.x) / r.d.x);
    double tx_max = std::max((max.x - r.o.x) / r.d.x, (min.x - r.o.x) / r.d.x);
    double ty_min = std::min((min.y - r.o.y) / r.d.y, (max.y - r.o.y) / r.d.y);
    double ty_max = std::max((min.y - r.o.y) / r.d.y, (max.y - r.o.y) / r.d.y);
    double tz_min = std::min((min.z - r.o.z) / r.d.z, (max.z - r.o.z) / r.d.z);
    double tz_max = std::max((min.z - r.o.z) / r.d.z, (max.z - r.o.z) / r.d.z);

    double t_enter = std::max({tx_min, ty_min, tz_min});
    double t_exit = std::min({tx_max, ty_max, tz_max});

    if (t_enter < t_exit && t_exit >= 0)
    {
      t0 = t_enter;
      t1 = t_exit;
      return true;
    }

    return false;
  }

  void BBox::draw(Color c, float alpha) const
  {

    glColor4f(c.r, c.g, c.b, alpha);

    // top
    glBegin(GL_LINE_STRIP);
    glVertex3d(max.x, max.y, max.z);
    glVertex3d(max.x, max.y, min.z);
    glVertex3d(min.x, max.y, min.z);
    glVertex3d(min.x, max.y, max.z);
    glVertex3d(max.x, max.y, max.z);
    glEnd();

    // bottom
    glBegin(GL_LINE_STRIP);
    glVertex3d(min.x, min.y, min.z);
    glVertex3d(min.x, min.y, max.z);
    glVertex3d(max.x, min.y, max.z);
    glVertex3d(max.x, min.y, min.z);
    glVertex3d(min.x, min.y, min.z);
    glEnd();

    // side
    glBegin(GL_LINES);
    glVertex3d(max.x, max.y, max.z);
    glVertex3d(max.x, min.y, max.z);
    glVertex3d(max.x, max.y, min.z);
    glVertex3d(max.x, min.y, min.z);
    glVertex3d(min.x, max.y, min.z);
    glVertex3d(min.x, min.y, min.z);
    glVertex3d(min.x, max.y, max.z);
    glVertex3d(min.x, min.y, max.z);
    glEnd();
  }

  std::ostream &operator<<(std::ostream &os, const BBox &b)
  {
    return os << "BBOX(" << b.min << ", " << b.max << ")";
  }

} // namespace CGL

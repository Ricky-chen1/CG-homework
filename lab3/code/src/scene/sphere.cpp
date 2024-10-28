#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  // 定义光线和球体联立方程参数
  double a = dot(r.d, r.d);
  double b = 2 * dot(r.o - o, r.d);
  double c = dot(r.o - o, r.o - o) - r2;

  double delta = b * b - 4 * a * c;
  if (delta < 0) {
    return false;
  }
  t1 = (-b - sqrt(delta)) / (2 * a);
  t2 = (-b + sqrt(delta)) / (2 * a);

  if(t2 < r.min_t || t1 > r.max_t){
    return false;
  }

  return true;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1, t2;
  if (!test(r,t1,t2))
  {
    return false;
  }

  return true;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  // t2有效
  double t1,t2;
  if(!test(r,t1,t2)){
    return false;
  }

  if(t2 <= r.max_t && t2 >= r.min_t){
    r.max_t = t2;
  }

  // t1有效（覆盖t2）
  if(t1 <= r.max_t && t1 >= r.min_t){
    r.max_t = t1;
  }

  i->bsdf = get_bsdf();
  i->primitive = this;
  i->t = r.max_t;
  // 归一化法向量
  i->n = (r.o + r.max_t * r.d - o).unit();
  return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL

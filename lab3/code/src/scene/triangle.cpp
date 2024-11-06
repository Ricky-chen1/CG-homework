#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL
{
  namespace SceneObjects
  {

    Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3)
    {
      p1 = mesh->positions[v1];
      p2 = mesh->positions[v2];
      p3 = mesh->positions[v3];
      n1 = mesh->normals[v1];
      n2 = mesh->normals[v2];
      n3 = mesh->normals[v3];
      bbox = BBox(p1);
      bbox.expand(p2);
      bbox.expand(p3);

      bsdf = mesh->get_bsdf();
    }

    BBox Triangle::get_bbox() const { return bbox; }

    bool Triangle::has_intersection(const Ray &r) const
    {
      // Part 1, Task 3: implement ray-triangle intersection
      // The difference between this function and the next function is that the next
      // function records the "intersection" while this function only tests whether
      // there is a intersection.
      // 向量应该逆时针顺序
      Vector3D ab = p2 - p1;
      Vector3D ac = p1 - p3; // 不应该为p3 - p1
      Vector3D bc = p3 - p2;
      double area = cross(ab, ac).norm() / 2;
      Vector3D face_normal = cross(ab, ac).unit();
      // 计算 t
      double t = dot(p1 - r.o, face_normal) / dot(r.d, face_normal);

      if (t < r.min_t || t > r.max_t)
      { // 检查 t 是否有效
        return false;
      }

      // 判断交点是否在三角形内
      Vector3D p = r.o + t * r.d;
      Vector3D ap = p - p1;
      Vector3D bp = p - p2;
      Vector3D cp = p - p3;

      // 计算叉积
      double cross1 = cross(ab, ap).z;
      double cross2 = cross(ac, cp).z;
      double cross3 = cross(bc, bp).z;

      if ((cross1 >= 0 && cross2 >= 0 && cross3 >= 0) ||
          (cross1 <= 0 && cross2 <= 0 && cross3 <= 0))
      {
        return true;
      }

      return false;
    }

    bool Triangle::intersect(const Ray &r, Intersection *isect) const
    {
      // Part 1, Task 3:
      // implement ray-triangle intersection. When an intersection takes
      // place, the Intersection data should be updated accordingly
      if (!has_intersection(r))
      {
        return false;
      }

      Vector3D ab = p2 - p1;
      Vector3D ac = p3 - p1;
      Vector3D bc = p3 - p2;
      double area = cross(ab, ac).norm() / 2;
      Vector3D face_normal = cross(ab, ac).unit();
      double t = dot(r.o - p1, face_normal) / dot(r.d, face_normal);
      Vector3D p = r.o + t * r.d;
      Vector3D ap = p - p1;
      Vector3D bp = p - p2;
      Vector3D cp = p - p3;

      double alpha = cross(ap, bp).norm() / 2 / area;
      double beta = cross(ap, cp).norm() / 2 / area;
      double gama = cross(bp, cp).norm() / 2 / area;

      if(t >= r.min_t && t <= r.max_t){
        r.max_t = t;
      }
      isect->bsdf = get_bsdf();
      isect->t = r.max_t;
      isect->primitive = this;
      isect->n = (alpha * n1 + beta * n2 + (1 - alpha - beta) * n3).unit();
      return true;
    }

    void Triangle::draw(const Color &c, float alpha) const
    {
      glColor4f(c.r, c.g, c.b, alpha);
      glBegin(GL_TRIANGLES);
      glVertex3d(p1.x, p1.y, p1.z);
      glVertex3d(p2.x, p2.y, p2.z);
      glVertex3d(p3.x, p3.y, p3.z);
      glEnd();
    }

    void Triangle::drawOutline(const Color &c, float alpha) const
    {
      glColor4f(c.r, c.g, c.b, alpha);
      glBegin(GL_LINE_LOOP);
      glVertex3d(p1.x, p1.y, p1.z);
      glVertex3d(p2.x, p2.y, p2.z);
      glVertex3d(p3.x, p3.y, p3.z);
      glEnd();
    }

  } // namespace SceneObjects
} // namespace CGL

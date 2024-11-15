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
      Vector3D E1 = p2 - p1;
      Vector3D E2 = p3 - p1;
      Vector3D S = r.o - p1;
      Vector3D S1 = cross(r.d, E2);
      Vector3D S2 = cross(S, E1);

      double S1E1 = dot(E1, S1);
      double b1 = dot(S, S1) / S1E1;
      double b2 = dot(r.d, S2) / S1E1;

      // 检查三个重心坐标是否在有效范围内
      if (b1 < 0.0 || b1 > 1.0 || b2 < 0.0 || b2 > 1.0 || b1 + b2 < 0.0 || b1 + b2 > 1.0)
      {
        return false;
      }

      // 计算交点参数 t
      double t = dot(E2, S2) / S1E1;

      // 检查 t 是否在光线的有效范围内
      return (t >= r.min_t && t <= r.max_t);
    }

    bool Triangle::intersect(const Ray &r, Intersection *isect) const
    {
      // Part 1, Task 3:
      // implement ray-triangle intersection. When an intersection takes
      // place, the Intersection data should be updated accordingly
      Vector3D E1 = p2 - p1;
      Vector3D E2 = p3 - p1;
      Vector3D S = r.o - p1;
      Vector3D S1 = cross(r.d, E2);
      Vector3D S2 = cross(S, E1);

      double S1E1 = dot(E1, S1);
      double b1 = dot(S, S1) / S1E1;
      double b2 = dot(r.d, S2) / S1E1;

      // 检查三个重心坐标是否在有效范围内
      if (b1 < 0.0 || b1 > 1.0 || b2 < 0.0 || b2 > 1.0 || b1 + b2 < 0.0 || b1 + b2 > 1.0)
      {
        return false;
      }

      // 计算交点参数 t
      double t = dot(E2, S2) / S1E1;

      // 检查 t 是否在光线的有效范围内
      if (t < r.min_t || t > r.max_t)
      {
        return false;
      }

      // 更新光线的最大有效距离 r.max_t
      r.max_t = t;
      // 更新交点信息
      isect->t = t;
      isect->primitive = this;
      isect->bsdf = get_bsdf();
      // 使用重心坐标插值法线并归一化
      isect->n = (b1 * n1 + b2 * n2 + (1 - b1 - b2) * n3).unit();

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

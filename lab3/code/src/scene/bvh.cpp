#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL
{
  namespace SceneObjects
  {

    BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                       size_t max_leaf_size)
    {

      primitives = std::vector<Primitive *>(_primitives);
      root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
    }

    BVHAccel::~BVHAccel()
    {
      if (root)
        delete root;
      primitives.clear();
    }

    BBox BVHAccel::get_bbox() const { return root->bb; }

    void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const
    {
      if (node->isLeaf())
      {
        for (auto p = node->start; p != node->end; p++)
        {
          (*p)->draw(c, alpha);
        }
      }
      else
      {
        draw(node->l, c, alpha);
        draw(node->r, c, alpha);
      }
    }

    void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const
    {
      if (node->isLeaf())
      {
        for (auto p = node->start; p != node->end; p++)
        {
          (*p)->drawOutline(c, alpha);
        }
      }
      else
      {
        drawOutline(node->l, c, alpha);
        drawOutline(node->r, c, alpha);
      }
    }

    BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                     std::vector<Primitive *>::iterator end,
                                     size_t max_leaf_size)
    {

      // TODO (Part 2.1):
      // Construct a BVH from the given vector of primitives and maximum leaf
      // size configuration. The starter code build a BVH aggregate with a
      // single leaf node (which is also the root) that encloses all the
      // primitives.

      BBox bbox;
      // 当前包围盒图元数
      size_t count = 0;

      // 获得整个包围盒
      for (auto p = start; p != end; p++)
      {
        BBox bb = (*p)->get_bbox();
        bbox.expand(bb);
        count++;
      }

      BVHNode *node = new BVHNode(bbox);
      // leaf node
      if (count <= max_leaf_size)
      {
        node->start = start;
        node->end = end;
        node->l = NULL;
        node->r = NULL;
        return node;
      }

      // 选择分割轴
      Vector3D extent = bbox.extent;
      int axis;
      if (extent.x >= extent.y && extent.x >= extent.z)
        axis = 0; // x 轴
      else if (extent.y >= extent.z)
        axis = 1; // y 轴
      else
        axis = 2; // z 轴

      // 计算图元的平均质心（用于分割）
      double split_point = 0;
      for (auto p = start; p != end; p++)
      {
        split_point += (*p)->get_bbox().centroid()[axis];
      }
      split_point /= count;

      // internal node
      // 划分左右
      auto mid = std::partition(start, end, [axis, split_point](Primitive *prim)
                                { return prim->get_bbox().centroid()[axis] < split_point; });

      // 无法根据质心划分时
      if (mid == start || mid == end)
      {
        return node;
      }

      node->l = construct_bvh(start, mid, max_leaf_size);
      node->r = construct_bvh(mid, end, max_leaf_size);

      return node;
    }

    bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const
    {
      // TODO (Part 2.3):
      // Fill in the intersect function.
      // Take note that this function has a short-circuit that the
      // Intersection version cannot, since it returns as soon as it finds
      // a hit, it doesn't actually have to find the closest hit.
      // 与当前节点的包围盒都不相交
      double t0 = ray.min_t, t1 = ray.max_t;
      if (!node->bb.intersect(ray, t0, t1))
      {
        return false;
      }

      if (node->isLeaf())
      {
        for (auto p = node->start; p != node->end; p++)
        {
          // 当前包围盒图元与光线相交
          if ((*p)->has_intersection(ray))
          {
            return true;
          }
        }
      }
      // 递归左右子树BVH
      return has_intersection(ray, node->l) || has_intersection(ray, node->r);
    }

    bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const
    {
      // TODO (Part 2.3):
      // Fill in the intersect function.
      double t0 = ray.min_t, t1 = ray.max_t;
      if (!node->bb.intersect(ray, t0, t1))
      {
        return false;
      }

      if (node->isLeaf())
      {
        bool hit = false;
        for (auto p = node->start; p != node->end; p++)
        {
          total_isects++;
          // 图元求交保证每次i都是最近交点,因而不需要在BVH中更新最近交点
          if ((*p)->intersect(ray, i))
          {
            hit = true;
          }
        }
        return hit;
      }

      bool hit = false;
      if (node->l->bb.intersect(ray, t0, t1))
      {
        hit = intersect(ray, i, node->l);
      }
      if (node->r->bb.intersect(ray, t0, t1))
      {
        hit = intersect(ray, i, node->r) || hit;
      }
      return hit;
    }
  } // namespace SceneObjects
} // namespace CGL

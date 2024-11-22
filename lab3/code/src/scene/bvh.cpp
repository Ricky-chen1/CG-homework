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

      // 初始化根节点
      BVHNode *root = new BVHNode(BBox());
      root->start = start;
      root->end = end;

      // 初始化整个包围盒
      for (auto p = start; p != end; p++)
      {
        root->bb.expand((*p)->get_bbox());
      }

      // 栈用于替代递归
      std::stack<BVHNode *> stack;
      stack.push(root);

      while (!stack.empty())
      {
        BVHNode *node = stack.top();
        stack.pop();

        // 计算当前节点包含的图元数量
        size_t count = std::distance(node->start, node->end);

        // 如果满足叶子节点条件，直接设置为叶子节点
        if (count <= max_leaf_size)
        {
          node->l = nullptr;
          node->r = nullptr;
          continue;
        }

        // 选择分割轴
        Vector3D extent = node->bb.extent;
        int axis = (extent.x >= extent.y && extent.x >= extent.z) ? 0 : (extent.y >= extent.z) ? 1
                                                                                               : 2;

        // 计算质心的平均值
        double split_point = 0;
        for (auto p = node->start; p != node->end; p++)
        {
          split_point += (*p)->get_bbox().centroid()[axis];
        }
        split_point /= count;

        auto mid = std::partition(node->start, node->end, [axis, split_point](Primitive *prim)
                                  { return prim->get_bbox().centroid()[axis] < split_point; });

        // 如果无法划分，直接标记为叶子节点
        if (mid == node->start || mid == node->end)
        {
          node->l = nullptr;
          node->r = nullptr;
          continue;
        }

        // 创建左右子树
        BVHNode *left = new BVHNode(BBox());
        BVHNode *right = new BVHNode(BBox());

        // 更新左右节点的包围盒
        for (auto p = node->start; p != mid; p++)
        {
          left->bb.expand((*p)->get_bbox());
        }
        for (auto p = mid; p != node->end; p++)
        {
          right->bb.expand((*p)->get_bbox());
        }

        left->start = node->start;
        left->end = mid;
        right->start = mid;
        right->end = node->end;

        // 连接子节点到当前节点
        node->l = left;
        node->r = right;

        // 将左右子节点压入栈中，继续处理
        stack.push(left);
        stack.push(right);
      }

      return root;
    }

    bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const
    {
      // TODO (Part 2.3):
      // Fill in the intersect function.
      // Take note that this function has a short-circuit that the
      // Intersection version cannot, since it returns as soon as it finds
      // a hit, it doesn't actually have to find the closest hit.
      // 与当前节点的包围盒都不相交
      std::stack<BVHNode *> stack;
      stack.push(node);

      while (!stack.empty())
      {
        BVHNode *node = stack.top();
        stack.pop();

        double t0 = ray.min_t, t1 = ray.max_t;
        if (!node->bb.intersect(ray, t0, t1))
        {
          continue;
        }

        // 如果是叶子节点，逐个检测图元
        if (node->isLeaf())
        {
          for (auto p = node->start; p != node->end; p++)
          {
            if ((*p)->has_intersection(ray))
            {
              return true; // 只需找到任意交点
            }
          }
        }
        else
        {
          // 非叶子节点，将子节点压入栈
          if (node->l)
            stack.push(node->l);
          if (node->r)
            stack.push(node->r);
        }
      }

      return false;
    }

    bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const
    {
      // TODO (Part 2.3):
      // Fill in the intersect function.
      std::stack<BVHNode *> stack;
      stack.push(node);
      bool hit = false;

      while (!stack.empty())
      {
        BVHNode *node = stack.top();
        stack.pop();

        // 当前节点与光线无交点，跳过
        double t0 = ray.min_t, t1 = ray.max_t;
        if (!node->bb.intersect(ray, t0, t1))
        {
          continue;
        }

        // 如果是叶子节点，逐个检测图元
        if (node->isLeaf())
        {
          for (auto p = node->start; p != node->end; p++)
          {
            if ((*p)->intersect(ray, i))
            {
              hit = true;
            }
          }
        }
        else
        {
          if (node->l)
            stack.push(node->l);
          if (node->r)
            stack.push(node->r);
        }
      }
      return hit;
    }
  } // namespace SceneObjects
} // namespace CGL

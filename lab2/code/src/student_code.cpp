#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    if (points.size() <= 1){
      return points;
    }
    vector<Vector2D>res;
    for(int i = 0;i < points.size() - 1;i ++){
      Vector2D newPoint = Vector2D(0,0);
      newPoint.x = t * points[i].x + (1 - t) * points[i + 1].x;
      newPoint.y = t * points[i].y + (1 - t) * points[i + 1].y;
      res.push_back(newPoint);
    }
    evaluateStep(res);
    return res;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    if (points.size() <= 1){
      return points;
    }
    vector<Vector3D> res;
    for (int i = 0;i < points.size() - 1;i ++){
      Vector3D newPoint = Vector3D(0,0,0);
      newPoint.x = points[i].x * t + points[i + 1].x * (1 - t);
      newPoint.y = points[i].y * t + points[i + 1].y * (1 - t);
      newPoint.z = points[i].z * t + points[i + 1].z * (1 - t);
      res.push_back(newPoint);
    }
    evaluateStep(res,t);
    return res;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    // 不需要保存中间控制点
    vector<Vector3D>res = points;
    while(res.size() > 1){
      res = evaluateStep(res,t);
    }
    return res[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    vector<Vector3D> row_points;
    Vector3D res;
    for(int i = 0;i < controlPoints.size();i ++){
      vector<Vector3D> points = controlPoints[i];
      Vector3D row_point = evaluate1D(points,u);
      row_points.push_back(row_point);
    }
    res = evaluate1D(row_points,v);
    return res;
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    Vector3D v_normal;
    // 该点对应的半边
    HalfedgeCIter h = this -> halfedge();
    do{
      FaceCIter f = h -> face();
      Vector3D a_pos = h ->vertex() -> position;
      //得到面的法线
      Vector3D ab,ac;
      h = h -> next();
      ab = h -> vertex() -> position - a_pos;
      h = h -> next();
      ac = h -> vertex() -> position - a_pos;
      h = h -> next();
      Vector3D face_normal = cross(ab,ac);
      double weight = face_normal.norm() / 2;
      // 加权平均（权重为面积）
      v_normal += weight * face_normal;
      // 遍历到下个包含该顶点的面
      h = h -> twin() -> next();
    }while(h != this -> halfedge());
    return v_normal.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    // 边界上的边
    if (e0 -> isBoundary()) {
      return e0;
    }
    
    return EdgeIter();
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    return VertexIter();
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.

  }
}

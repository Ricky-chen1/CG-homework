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
    // 边界上的边不应该翻转
    if (e0 -> isBoundary()) {
      return e0;
    }
    // before flip
    // halfedges（从e0开始构造）
    HalfedgeIter h0 = e0 -> halfedge();
    HalfedgeIter h1 = h0 -> next(); 
    HalfedgeIter h2 = h1 -> next();
    HalfedgeIter h3 = h0 -> twin();
    HalfedgeIter h4 = h3 -> next();
    HalfedgeIter h5 = h4 -> next();
    HalfedgeIter h6 = h1 -> twin();
    HalfedgeIter h7 = h2 -> twin();
    HalfedgeIter h8 = h4 -> twin();
    HalfedgeIter h9 = h5 -> twin();
    
    // vertexs
    VertexIter v0 = h0 -> vertex();
    VertexIter v1 = h3 -> vertex();
    VertexIter v2 = h2 -> vertex();
    VertexIter v3 = h5 -> vertex();

    // edges
    EdgeIter e1 = h1 -> edge();
    EdgeIter e2 = h2 -> edge();
    EdgeIter e3 = h4 -> edge();
    EdgeIter e4 = h5 -> edge(); 

    // faces
    FaceIter f0 = h0 -> face();
    FaceIter f1 = h3 -> face();

    // after flip
    // halfedges
    // outside元素的next和face应该不变
    h0 -> setNeighbors(h1,h3,v3,e0,f0);
    h1 -> setNeighbors(h2,h7,v2,e2,f0);
    h2 -> setNeighbors(h0,h8,v0,e3,f0);
    h3 -> setNeighbors(h4,h0,v2,e0,f1);
    h4 -> setNeighbors(h5,h9,v3,e4,f1);
    h5 -> setNeighbors(h3,h6,v1,e1,f1);
    h6 -> setNeighbors(h6 -> next(),h5,v2,e1,h6 -> face());
    h7 -> setNeighbors(h7 -> next(),h1,v0,e2,h7 -> face());
    h8 -> setNeighbors(h8 -> next(),h2,v3,e3,h8 -> face());
    h9 -> setNeighbors(h9 -> next(),h4,v1,e4,h9 -> face());

    // vertexs(有多条半边可选，需要能够遍历一个面)
    v0 -> halfedge() = h2;
    v1 -> halfedge() = h5;
    v2 -> halfedge() = h3;
    v3 -> halfedge() = h0;

    // edges
    e0 -> halfedge() = h0;
    e1 -> halfedge() = h5;
    e2 -> halfedge() = h1;
    e3 -> halfedge() = h2;
    e4 -> halfedge() = h4;

    // faces
    f0 -> halfedge() = h0;
    f1 -> halfedge() = h3;

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    // before split
    // halfedges（从e0开始构造）
    if (e0 -> isBoundary()){
      return e0 -> halfedge() -> vertex();
    }

    HalfedgeIter h0 = e0 -> halfedge();
    HalfedgeIter h1 = h0 -> next(); 
    HalfedgeIter h2 = h1 -> next();
    HalfedgeIter h3 = h0 -> twin();
    HalfedgeIter h4 = h3 -> next();
    HalfedgeIter h5 = h4 -> next();
    HalfedgeIter h6 = h1 -> twin();
    HalfedgeIter h7 = h2 -> twin();
    HalfedgeIter h8 = h4 -> twin();
    HalfedgeIter h9 = h5 -> twin();
    
    // vertexs
    VertexIter v0 = h0 -> vertex();
    VertexIter v1 = h3 -> vertex();
    VertexIter v2 = h2 -> vertex();
    VertexIter v3 = h5 -> vertex();

    // edges
    EdgeIter e1 = h1 -> edge();
    EdgeIter e2 = h2 -> edge();
    EdgeIter e3 = h4 -> edge();
    EdgeIter e4 = h5 -> edge(); 

    // faces
    FaceIter f0 = h0 -> face();
    FaceIter f1 = h3 -> face();

    // new elements
    HalfedgeIter h10 = newHalfedge();
    HalfedgeIter h11 = newHalfedge();
    HalfedgeIter h12 = newHalfedge();
    HalfedgeIter h13 = newHalfedge();
    HalfedgeIter h14 = newHalfedge();
    HalfedgeIter h15 = newHalfedge();

    VertexIter v4 = newVertex();

    EdgeIter e5 = newEdge();
    EdgeIter e6 = newEdge();
    EdgeIter e7 = newEdge();

    FaceIter f2 = newFace();
    FaceIter f3 = newFace();

    // after split
    h0 -> setNeighbors(h1,h3,v4,e0,f0);
    h1 -> setNeighbors(h11,h6,v1,e1,f0);
    h2 -> setNeighbors(h14,h7,v2,e2,f2);
    h3 -> setNeighbors(h10,h0,v1,e0,f1);
    h4 -> setNeighbors(h13,h8,v0,e3,f3);
    h5 -> setNeighbors(h3,h9,v3,e4,f1);
    // 6,7,8,9所有元素不变
    h10 -> setNeighbors(h5,h13,v4,e5,f1);
    h11 -> setNeighbors(h0,h15,v2,e7,f0);
    h12 -> setNeighbors(h4,h14,v4,e6,f3);
    h13 -> setNeighbors(h12,h10,v3,e5,f3);
    h14 -> setNeighbors(h15,h12,v0,e6,f2);
    h15 -> setNeighbors(h2,h11,v4,e7,f2);

    v0 -> halfedge() = h14;
    v1 -> halfedge() = h3;
    v2 -> halfedge() = h2;
    v3 -> halfedge() = h5;
    v4 -> halfedge() = h0;

    e5 -> halfedge() = h13;
    e6 -> halfedge() = h14;
    e7 -> halfedge() = h15;

    f0 -> halfedge() = h0;
    f1 -> halfedge() = h3;
    f2 -> halfedge() = h14;
    f3 -> halfedge() = h12;

    v4->position = (v1->position + v0->position) / 2;
    return v4;
  }


  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    for(VertexIter v = mesh.verticesBegin();v != mesh.verticesEnd();v ++){
      // 对每个原始顶点都计算新的位置
      v -> isNew = false;
      Vector3D total_pos(0,0,0);
      int n = 0;
      double u;
      HalfedgeCIter h = v -> halfedge();
      do{
        HalfedgeCIter h_twin = h -> twin();
        VertexCIter v = h_twin -> vertex();
        total_pos += v -> position;
        n ++;
        h = h_twin -> next();
      }while(h != v -> halfedge());

      if (n == 3){
        u = 3.0 / 16.0;
      }else{
        u = 3.0 / (8.0 * n);
      }

      Vector3D new_pos = (1 - n * u) * v -> position + u * total_pos;
      v -> newPosition = new_pos;
      v -> isNew = false;
    }
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    for(EdgeIter e = mesh.edgesBegin();e != mesh.edgesEnd();e ++){
      HalfedgeIter h = e -> halfedge();
      VertexIter v0 = h -> vertex(); // A
      VertexIter v1 = h -> twin() -> vertex(); // B
      VertexIter v2 = h -> next() -> twin() -> vertex(); // C
      VertexIter v3 = h -> twin() -> next() -> twin() -> vertex(); // D

      e -> newPosition =  (v0 -> position + v1 -> position) * 3.0 / 8.0 + (v2 -> position + v3 -> position) * 1.0 / 8.0;
    }

    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    vector<EdgeIter> originEdgeList;
    EdgeIter e = mesh.edgesBegin();
    while (e != mesh.edgesEnd()) {
      EdgeIter nowEdge = e;
      nowEdge -> isNew = false;
      originEdgeList.push_back(nowEdge);
      e ++;
    }

    for(auto originEdge = originEdgeList.begin();originEdge != originEdgeList.end();originEdge ++){
      // 分割原始网格每条边
      auto v = mesh.splitEdge(*originEdge);
      v -> isNew = true;
      v -> newPosition = (*originEdge) -> newPosition;

      // 设置新边flag
      auto newHalfedge = v -> halfedge();
      newHalfedge -> edge() -> isNew = false;
      newHalfedge -> twin() -> next() -> edge() -> isNew = true;
      newHalfedge -> twin() -> next() -> twin() -> next() -> edge() -> isNew = false;
      newHalfedge -> next() -> next() -> edge() -> isNew = true;
    }

    // 4. Flip any new edge that connects an old and new vertex.
     for(EdgeIter e = mesh.edgesBegin();e != mesh.edgesEnd();e ++){
      HalfedgeIter h = e -> halfedge();
      VertexIter v0 = h -> vertex();
      VertexIter v1 = h -> twin() -> vertex();
      // 翻转连接新旧顶点的新边
      if((v0 -> isNew ^ v1 -> isNew) && e -> isNew){
        mesh.flipEdge(e);
      }
    }

    // 5. Copy the new vertex positions into final Vertex::position.
    for(VertexIter v = mesh.verticesBegin();v != mesh.verticesEnd();v ++){
      v -> position = v -> newPosition;
    }
  }
}

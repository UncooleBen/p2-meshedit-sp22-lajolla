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
    Vector2D interPoint;
    std::vector<Vector2D> interPoints;
    //
    for (int point_id= 1; point_id < points.size(); point_id++){
      interPoint.x = points[point_id-1].x*(1-t) + points[point_id].x*t;
      interPoint.y = points[point_id-1].y*(1-t) + points[point_id].y*t;
      interPoints.push_back(interPoint);
    }
    return interPoints;
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
    //BezierPatch::evaluateStep(...): Very similar to BezierCurve::evaluateStep(...) in Part 1, 
    //this recursive function takes as inputs a std::vector of 3D points and a parameter tt. 
    //It outputs a std::vector of intermediate control points at the parameter t in the next subdivision level.
    std::vector<Vector3D> interPoints;
    for (int point_id= 1; point_id < points.size(); point_id++){
      interPoints.push_back(points[point_id-1]*(1-t) + points[point_id]*t);
    }
    return interPoints;
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
    //This function takes as inputs a std::vector of 3D points and a parameter tt.
    // It outputs directly the final, single point that lies on the Bezier curve at the parameter tt. 
    //This function does not output intermediate control points. 
    //You may want to call BezierPatch::evaluateStep(...) inside this function.
    std::vector<Vector3D> lastStepPoint = points;
    while (lastStepPoint.size() >= 1) {
      lastStepPoint = evaluateStep(lastStepPoint, t);
    }
    return lastStepPoint[0];
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
    //This function takes as inputs the parameter uu and vv and outputs the point that lies on the Bezier surface, 
    //defined by the n \times nn×n controlPoints, at the parameter uu and vv.
    // Note that controlPoints is a member variable of the BezierPatch class, 
    //which you have access to within this function.
    vector<Vector3D> curves_u;
    for (int row=0; row<controlPoints.size(); row++) {
    Vector3D curvPoint = evaluate1D(controlPoints[row], u);
    //curves_u[row] = curvPoint;
    curves_u.push_back(curvPoint);
    }
    return evaluate1D(curves_u, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    return Vector3D();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
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

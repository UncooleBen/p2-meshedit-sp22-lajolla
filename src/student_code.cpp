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
    std::vector<Vector3D> current_points = points;
    while (current_points.size() > 1) {
      current_points = evaluateStep(current_points, t);
    }
    return current_points[0];
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
    std::vector<Vector3D> column_points(controlPoints.size(), Vector3D());
    for (int row=0; row<controlPoints.size(); row++) {
      Vector3D column_point = evaluate1D(controlPoints[row], u);
      column_points[row] = column_point;
    }
    return evaluate1D(column_points, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    Vector3D normal;
    HalfedgeCIter iter = halfedge();
    do {
      Vector3D va = iter->vertex()->position;
      Vector3D vb = iter->next()->vertex()->position;
      Vector3D vc = iter->next()->next()->vertex()->position;
      // length of cross product is proportional to triangle area
      Vector3D cross_product = CGL::cross(vb - va, vc - va);
      normal += cross_product;
      iter = iter->twin()->next();
    } while (iter != halfedge());
    return normal.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    //delete one edge add new one edge
    //delete two halfedges add two new halfedges
    //change four vertex
    //might change face?

    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h3 = e0->halfedge()->twin();
    //return if either neighbouring face of the edge is on a boundary loop. 
    if(e0->isBoundary() ||  h0->face()->isBoundary()|| h0->twin()->face()->isBoundary() ){
      return e0;
    }
    VertexIter vert0  = h0->vertex();
    VertexIter vert1  = h3->vertex();
    VertexIter vert3  = h0->next()->next()->vertex();
    VertexIter vert2  = h3->next()->next()->vertex();
    //e1/2/3/4 don't change
    HalfedgeIter h1 =  h0->next();
    HalfedgeIter h2 =  h1->next();
    HalfedgeIter h4 =  h3->next();
    HalfedgeIter h5 =  h4->next();
    FaceIter face0 = h0->face();
    FaceIter face1 = h3->face();
    face0->halfedge() = h2;
    face1->halfedge() = h1;
    // // no new here

    h2->next() = h4; h4->next() = h0; h0->next() = h2;
    h5->next() = h1; h1->next() = h3; h3->next() = h5; 
    h0->vertex() = vert2; h0->face()= face0; h0->twin() = h3;
    h3->vertex() = vert3; h3->face()= face1; h3->twin() = h0;
    //update the change face for each halfedge;
    h2->face() = face0; h4->face() = face0;
    h1->face() = face1; h5->face() = face1;

    vert0->halfedge() = h4;
    vert1->halfedge() = h1;
    vert2->halfedge() = h0;
    vert3->halfedge() = h3;
    e0->halfedge() = h0;
    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    //Draw a simple mesh, such as the pair of triangles (a,b,c)(a,b,c) and (c,b,d)(c,b,d) above, and write down a list of all elements, i.e., half-edges, vertices, edges, and faces, in this mesh.
    //Draw the mesh after the remeshing operation and again write down a list of all elements in the now modified mesh.
    //For every element in the modified mesh, set all of its pointers to the correct element in the modified mesh, even if the element being pointed to has not changed:
    
    //For each vertex, edge, and face, set its halfedge pointer.
    //For each half-edge, set its next, twin, vertex, edge, and face pointer to the correct element. You can use Halfedge::setNeighbors(...) to set all pointers of a half-edge at once.
  
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h3 = e0->halfedge()->twin();
    HalfedgeIter h1 =  h0->next();
    HalfedgeIter h2 =  h1->next();
    HalfedgeIter h4 =  h3->next();
    HalfedgeIter h5 =  h4->next();
    //return if either neighbouring face of the edge is on a boundary loop. 
    if (h0->face()->isBoundary() || h0->twin()->face()->isBoundary()){
      return VertexIter();
    } 
    // new mesh vertex  two new triangles, three new edges, 
    Vector3D vert0_pos  = h0->vertex()->position;
    Vector3D vert1_pos  = h3->vertex()->position;
    VertexIter verta  = h0->next()->next()->vertex();
    VertexIter vertd  = h3->next()->next()->vertex();
    FaceIter faceabc = h0->face();
    FaceIter facebdc = h3->face();

    Vector3D vertm_pos = (vert0_pos + vert1_pos)/2;
    VertexIter vertm = newVertex();
    vertm->isNew = true;
    vertm->position = vertm_pos;
    //new half_edge()
    HalfedgeIter h0_mc = newHalfedge();
    h0_mc->vertex() = vertm;
    h0_mc->next() = h0->next();
    HalfedgeIter h3_mb = newHalfedge(); 
    h3_mb->vertex() = vertm;
    h3_mb->next() = h3->next();
    HalfedgeIter h_am = newHalfedge();
    h_am -> vertex() = verta;
    HalfedgeIter h_ma = newHalfedge();
    h_ma -> vertex() = vertm;
    HalfedgeIter h_dm = newHalfedge();
    h_dm -> vertex() = vertd;
    HalfedgeIter h_md = newHalfedge();
    h_md -> vertex() = vertm;
    //new edges
    EdgeIter edge_bm = e0;
    EdgeIter edge_am = newEdge();
    EdgeIter edge_md = newEdge();
    EdgeIter edge_mc = newEdge();
    // two new faces
    FaceIter face_amc = newFace();
    FaceIter face_mdc = newFace();
    //vert edge face
    vertm->halfedge() = h3_mb;
    edge_am->halfedge() = h_am;
    edge_md->halfedge() = h_md;
    edge_mc->halfedge() = h0_mc;
    faceabc->halfedge() = h2;
    facebdc->halfedge() = h4;
    face_amc -> halfedge() = h1;
    face_mdc -> halfedge() = h5;

    //each half-edge
    h_am -> next() = h0_mc; h0_mc->next() = h1; h1->next() = h_am;
    h_am -> twin() = h_ma; h_am -> edge() = edge_am; h_am -> face() = face_amc;
    
    h_ma -> next() = h2; h2->next() = h0; h0->next() = h_ma;
    h_ma -> twin() = h_am; h_ma -> edge() = edge_am; h_ma -> face() = faceabc;

    h_md -> next() = h5; h5 -> next() = h3; h3 -> next() = h_md;
    h_md -> twin() = h_dm; h_md -> edge() = edge_md; h_md -> face() = face_mdc;

    h_dm -> next () = h3_mb; h3_mb->next() = h4; h4->next() = h_dm;
    h_dm -> twin() = h_md; h_dm -> edge() = edge_md; h_dm -> face() = facebdc;
    
    h0_mc -> twin() = h3; h0_mc -> edge() = edge_mc; h0_mc -> face() = face_amc; 

    h3_mb -> twin() = h0; h3_mb -> edge() = edge_bm; h3_mb -> face() = facebdc;
    
    h0 -> twin() = h3_mb; h0 -> face() = faceabc; h0 -> edge() = e0;

    h3 -> twin() = h0_mc; h3 -> face() = face_mdc; h3 -> edge()= edge_mc;
    return vertm;
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

    // Step 1
    // 1.A. all old vertices and compute newPosition of old vertices
    for (VertexIter iter=mesh.verticesBegin(); iter!=mesh.verticesEnd(); iter++) {
      HalfedgeIter current_half_edge = iter->halfedge();
      VertexIter center_vertex = current_half_edge->vertex();
      double n = center_vertex->degree();
      double u;
      if (n == 3) {
        u = 3.0 / 16;
      } else {
        u = 3.0 / 8 / n;
      }
      // Set new position's base value to (1 - n*u) * center_vertex_position
      center_vertex->newPosition = (1 - n * u) * center_vertex->position;
      do {
        VertexIter neighbor_vertex = current_half_edge->next()->vertex();
        // Accumulate neighbor old vertiex positions
        center_vertex->newPosition += neighbor_vertex->position * u;
        // Update loop variable
        current_half_edge = current_half_edge->twin()->next();
      } while (current_half_edge != iter->halfedge());
      // std::cout << center_vertex->newPosition << std::endl;
    }
    // 1.B. all old edges and compute position of new (to-be-created) vertices in edge's newPosition
    for (EdgeIter iter=mesh.edgesBegin(); iter!=mesh.edgesEnd(); iter++) {
      HalfedgeIter current_half_edge = iter->halfedge();
      // Set new position to zero for accumulation
      iter->newPosition = Vector3D();
      iter->newPosition += (3.0 / 8) * current_half_edge->vertex()->position;
      iter->newPosition += (3.0 / 8) * current_half_edge->next()->vertex()->position;
      iter->newPosition += (1.0 / 8) * current_half_edge->next()->next()->vertex()->position;
      iter->newPosition += (1.0 / 8) * current_half_edge->twin()->next()->next()->vertex()->position;
      // std::cout << iter->newPosition << std::endl;
    }

    // Step 2 loop subdivision via edge split and edge flip
    int n = mesh.nEdges();
    EdgeIter iter = mesh.edgesBegin();
    for (int tmp=0; tmp<n; tmp++) {
      EdgeIter next = std::next(iter);
      mesh.splitEdge(iter);
      iter = next;
    }

    // for (EdgeIter iter=mesh.edgesBegin(); iter!=mesh.edgesEnd(); iter++) {
    //   if (iter->halfedge()->vertex()->isNew ^ iter->halfedge()->next()->vertex()->isNew) {
    //     mesh.flipEdge(iter);
    //   }
    // }
    
    // Step 3 set new positions
    for (VertexIter iter=mesh.verticesBegin(); iter!=mesh.verticesEnd(); iter++) {
      if (iter->isNew) {
        iter->position = iter->halfedge()->edge()->newPosition;
      } else {
        iter->position = iter->newPosition;
      }
      cout << iter->position << endl;
      iter->newPosition = Vector3D();
      iter->isNew = false;
    }
  }
}

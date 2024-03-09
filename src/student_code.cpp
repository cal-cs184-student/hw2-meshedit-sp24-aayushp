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
      vector<Vector2D> fin = std::vector<Vector2D>();
      for (int i = 0; i < points.size() - 1; i ++) {
          fin.push_back((1-t)*points[i]+ (t* points[i+1]));
      }

      return fin;
//    return std::vector<Vector2D>();
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
      vector<Vector3D> fin = std::vector<Vector3D>();
//      if (points.size() > 0){
          for (int i = 0; i < points.size() - 1; i ++) {
              fin.push_back((1-t)*points[i]+ (t* points[i+1]));
          }
//      }


      return fin;
//    return std::vector<Vector3D>();
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
    vector<Vector3D> inter = points;
    while (inter.size() > 1) {
        inter = evaluateStep(inter, t);
    }
    Vector3D fin = inter.at(0);
    return fin;
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
//    Vector3D fin = Vector3D();
    vector<Vector3D> inter = std::vector<Vector3D>();
    for (int i = 0; i < controlPoints.size(); i ++) {
        inter.push_back(evaluate1D(controlPoints.at(i), u));
    };
//    for (int i = 0; i < inter.size(); i++) {
//
//    }
    Vector3D fin = evaluate1D(inter, v);
//    controlPoints
    return fin;
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    Vector3D fin = Vector3D();
    vector<Vector3D> vertices = vector<Vector3D>();
//    Vertex v = this;
    HalfedgeCIter h = this->halfedge();      // get the outgoing half-edge of the vertex
        do {
            HalfedgeCIter h_twin = h->twin(); // get the opposite half-edge
            VertexCIter v = h_twin->vertex(); // vertex is the 'source' of the half-edge, so
            // h->vertex() is v, whereas h_twin->vertex()
            // is the neighboring vertex
//            cout << v->position << endl;      // print the vertex position
            vertices.push_back(v->position); // add vertices to the vector;
            h = h_twin->next();               // move to the next outgoing half-edge of the vertex
        } while(h != this->halfedge());          // keep going until we are back where we were
    //for each half edge incident to the position
        // get next half edge until you reach the first half edge. Use the last iterated half edge and the first half
        //edge to find cross product (save the incident half edge, and then just get the second to last one) -- that is the normal
        // also as doing this, get list of vertices -- then calculate area
        Vector3D initial = vertices.at(0);
        Vector3D normal_sum = Vector3D();
        for (int i = 0; i < vertices.size(); i++) {
            if (i != vertices.size()-1){
                Vector3D vector_1 = vertices.at(i) -this->position;
                Vector3D vector_2 = this->position -  vertices.at(i+1);
                Vector3D cp = cross(vector_1, vector_2);
                normal_sum += cp;
            }
            else {
                Vector3D vector_1 = vertices.at(i) -this->position;
                Vector3D vector_2 = this-> position - initial;
                Vector3D cp = cross(vector_1, vector_2);
                normal_sum+= cp;
            }
        }
        fin = normal_sum/ normal_sum.norm();


//    void printNeighbourPositions(VertexCIter v) {
//        HalfEdgeCIter h = v->halfedge();      // get the outgoing half-edge of the vertex
//        do {
//            HalfEdgeCIter h_twin = h->twin(); // get the opposite half-edge
//            VertexCIter v = h_twin->vertex(); // vertex is the 'source' of the half-edge, so
//            // h->vertex() is v, whereas h_twin->vertex()
//            // is the neighboring vertex
//            cout << v->position << endl;      // print the vertex position
//            h = h_twin->next();               // move to the next outgoing half-edge of the vertex
//        } while(h != v->halfedge());          // keep going until we are back where we were
//    }
    return fin;
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
//      HalfedgeIter h0 = e0->halfedge();
//      HalfedgeIter h1 = h0->next();
//      HalfedgeIter h2 = h1->next();
//      HalfedgeIter h3 = h0->twin();
//      HalfedgeIter h4 = h3->next();
//      HalfedgeIter h5 = h4->next();
//      HalfedgeIter h6 = h1->twin();
//      HalfedgeIter h7 = h2->twin();
//      HalfedgeIter h8 = h4->twin();
//      HalfedgeIter h9 = h5->twin();

    if (!e0->isBoundary()) {

        HalfedgeIter h0 = e0->halfedge();
        HalfedgeIter h1 = h0->next();
        HalfedgeIter h2 = h1->next();
        HalfedgeIter h3 = h0->twin();
        HalfedgeIter h4 = h3->next();
        HalfedgeIter h5 = h4->next();
        HalfedgeIter h6 = h1->twin();
        HalfedgeIter h7 = h2->twin();
        HalfedgeIter h8 = h4->twin();
        HalfedgeIter h9 = h5->twin();

        VertexIter v0 = h0->vertex();
        VertexIter v1 = h3->vertex();
        VertexIter v2 = h2->vertex();
        VertexIter v3 = h5->vertex();

        EdgeIter e1 = h1->edge();
        EdgeIter e2 = h2->edge();
        EdgeIter e3 = h4->edge();
        EdgeIter e4 = h5->edge();

        FaceIter f0 = h0->face();
        FaceIter f1 = h3->face();

        h0->next() = h1;
        h0->twin() = h3;
        h0->edge() = e0;
        h0->vertex() = v3;
        h0->face() = f0;

        h1->next() = h2;
        h1->twin() = h7;
        h1->vertex() = v2;
        h1->edge() = e2;
        h1->face() = f0;

        h2->next() = h0;
        h2->twin() = h8;
        h2->vertex() = v0;
        h2->edge() = e3;
        h2->face() = f0;

        h3->next() = h4;
        h3->twin() = h0;
        h3->vertex() = v2;
        h3->edge() = e0;
        h3->face() = f1;

        h4->next() = h5;
        h4->twin() = h9;
        h4->vertex() = v3;
        h4->edge() = e4;
        h4->face() = f1;

        h5->next() = h3;
        h5->twin() = h6;
        h5->vertex() = v1;
        h5->edge() = e1;
        h5->face() = f1;

        h6->next() = h6->next();
        h6->twin() = h5;
        h6->vertex() = v2;
        h6->edge() = e1;
        h6->face() = h6->face();

        h7->next() = h7->next();
        h7->twin() = h1;
        h7->vertex() = v0;
        h7->edge() = e2;
        h7->face() = h7->face();

        h8->next() = h8->next();
        h8->twin() = h2;
        h8->vertex() = v3;
        h8->edge() = e3;
        h8->face() = h8->face();

        h9->next() = h9->next();
        h9->twin() = h4;
        h9->vertex() = v1;
        h9->edge() = e4;
        h9->face() = h9->face();

        v0->halfedge() = h2;
        v1->halfedge() = h5;
        v2->halfedge() = h3;
        v3->halfedge() = h0;

        e0->halfedge() = h0;
        e1->halfedge() = h5;
        e2->halfedge() = h1;
        e3->halfedge() = h2;
        e4->halfedge() = h4;

        f1->halfedge() = h0;
        f1->halfedge() = h3;

    }
    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    if (!e0->isBoundary()) {
        HalfedgeIter h0 = e0->halfedge();
        HalfedgeIter h1 = h0->next();
        HalfedgeIter h2 = h1->next();
        HalfedgeIter h3 = h0->twin();
        HalfedgeIter h4 = h3->next();
        HalfedgeIter h5 = h4->next();
        HalfedgeIter h6 = h1->twin();
        HalfedgeIter h7 = h2->twin();
        HalfedgeIter h8 = h4->twin();
        HalfedgeIter h9 = h5->twin();

        VertexIter v0 = h0->vertex();
        VertexIter v1 = h3->vertex();
        VertexIter v2 = h2->vertex();
        VertexIter v3 = h5->vertex();

        EdgeIter e1 = h1->edge();
        EdgeIter e2 = h2->edge();
        EdgeIter e3 = h4->edge();
        EdgeIter e4 = h5->edge();

        FaceIter f0 = h0->face();
        FaceIter f1 = h3->face();

        VertexIter v4 = newVertex();

        EdgeIter e5 = newEdge();
        EdgeIter e6 = newEdge();
        EdgeIter e7 = newEdge();

        HalfedgeIter h10 = newHalfedge();
        HalfedgeIter h11 = newHalfedge();
        HalfedgeIter h12 = newHalfedge();
        HalfedgeIter h13 = newHalfedge();
        HalfedgeIter h14 = newHalfedge();
        HalfedgeIter h15 = newHalfedge();

        FaceIter f2 = newFace();
        FaceIter f3 = newFace();


        //new vertex position
        v4->position = (v1->position + v0->position) / 2;

        h0->next() = h1;
        h0->twin() = h3;
        h0->edge() = e0;
        h0->vertex() = v0;
        h0->face() = f0;

        h1->next() = h2;
        h1->twin() = h15;
        h1->edge() = e5;
        h1->vertex() = v4;
        h1->face() = f0;

        h2->next() = h0;
        h2->twin() = h7;
        h2->edge() = e2;
        h2->vertex() = v2;
        h2->face() = f0;

        h3->next() = h4;
        h3->twin() = h0;
        h3->edge() = e0;
        h3->vertex() = v4;
        h3->face() = f1;

        h4->next() = h5;
        h4->twin() = h8;
        h4->edge() = e3;
        h4->vertex() = v0;
        h4->face() = f1;

        h5->next() = h3;
        h5->twin() = h10;
        h5->edge() = e6;
        h5->vertex() = v3;
        h5->face() = f1;

        h6->next() = h6->next();
        h6->twin() = h14;
        h6->vertex() = v2;
        h6->edge() = e1;
        h6->face() = h6->face();

        h7->next() = h7->next();
        h7->twin() = h2;
        h7->vertex() = v0;
        h7->edge() = e2;
        h7->face() = h7->face();

        h8->next() = h8->next();
        h8->twin() = h4;
        h8->vertex() = v3;
        h8->edge() = e3;
        h8->face() = h8->face();

        h9->next() = h9->next();
        h9->twin() = h11;
        h9->vertex() = v1;
        h9->edge() = e4;
        h9->face() = h9->face();

        h10->next() = h11;
        h10->twin() = h5;
        h10->vertex() = v4;
        h10->edge() = e6;
        h10->face() = f2;

        h11->next() = h12;
        h11->twin() = h9;
        h11->vertex() = v3;
        h11->edge() = e4;
        h11->face() = f2;

        h12->next() = h10;
        h12->twin() = h13;
        h12->vertex() = v1;
        h12->edge() = e7;
        h12->face() = f2;

        h13->next() = h14;
        h13->twin() = h12;
        h13->vertex() = v4;
        h13->edge() = e7;
        h13->face() = f3;

        h14->next() = h15;
        h14->twin() = h6;
        h14->vertex() = v1;
        h14->edge() = e1;
        h14->face() = f3;

        h15->next() = h13;
        h15->twin() = h1;
        h15->vertex() = v2;
        h15->edge() = e5;
        h15->face() = f3;

        v0->halfedge() = h4;
        v1->halfedge() = h14;
        v2->halfedge() = h2;
        v3->halfedge() = h5;
        v4->halfedge() = h3;

        e0->halfedge() = h0;
        e1->halfedge() = h14;
        e2->halfedge() = h2;
        e3->halfedge() = h4;
        e4->halfedge() = h11;
        e5->halfedge() = h15;
        e6->halfedge() = h10;
        e7->halfedge() = h12;

        f0->halfedge() = h0;
        f1->halfedge() = h3;
        f2->halfedge() = h10;
        f3->halfedge() = h13;

        e6->isNew = true;
        e5->isNew = true;
//        e0->isNew = true;
//        e7->isNew = true;

        v4->isNew = true;
//        CHECK_CLOSED

        return v4;
    } else {
        return VertexIter ();
    }
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
//          cout << v->position << endl; // do something interesting here
            HalfedgeIter h = v->halfedge();      // get the outgoing half-edge of the vertex
//            vector<Vector3D> adjacents = vector<Vector3D>();
            Vector3D sum = Vector3D();
              do {
                  HalfedgeIter h_twin = h->twin(); // get the opposite half-edge
                  VertexIter v2 = h_twin->vertex(); // vertex is the 'source' of the half-edge, so
                  // h->vertex() is v, whereas h_twin->vertex()
                  // is the neighboring vertex
//                  cout << v->position << endl;      // print the vertex position
//                  adjacents.push_back(v->position);
                  sum += v2->position;
                  h = h_twin->next();               // move to the next outgoing half-edge of the vertex
              } while(h != v->halfedge());          // keep going until we are back where we were
               Size degree = v->degree();
          if (degree != 3) {
              v->newPosition = (1.0f - degree * (3.0f / (8.0f * degree))) * v->position + (3.0f / (8.0f * degree)) * sum;
          } else {
              v->newPosition = (1.0f - degree * (3.0f / 16.0f)) * v->position + (3.0f / 16.0f) * sum;
//              (1 - n * u) * original_position + u * original_neighbor_position_sum
          }
          v->isNew = false;

//            v->newPosition = (1 - v->degree() * )
      }
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.

      EdgeIter e = mesh.edgesBegin();
      while (e != mesh.edgesEnd()) {

          // get the next edge NOW!
          EdgeIter nextEdge = e;
          nextEdge++;

          // now, even if splitting the edge deletes it ...
//          if (some condition is met) {
//              mesh.splitEdge(e);
//          }
//          3/8 * (A + B) + 1/8 * (C + D)
//          e->newPosition = (e->halfedge()->vertex()->position + e->halfedge()->twin()->vertex()->position)/2;
          e->newPosition = ((e->halfedge()->vertex()->position + e->halfedge()->twin()->vertex()->position) * (3.0f/8))
                  + ((1.0f/8) * (e->halfedge()->twin()->next()->next()->vertex()->position + e->halfedge()->next()->next()->vertex()->position));

//          e->getVertex()
            e->isNew = false;
          // ... we still have a valid reference to the next edge
          e = nextEdge;
      }


    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)

      e = mesh.edgesBegin();
      int n = mesh.nEdges();
      for (int i = 0; i < n; i++) {
//      while (e != mesh.edgesEnd()) {

          // get the next edge NOW!
          EdgeIter nextEdge = e;
          nextEdge++;

          // now, even if splitting the edge deletes it ...
//          if (some condition is met) {
//              mesh.splitEdge(e);
//          }
//          e->newPosition = (e->halfedge()->vertex()->position + e->halfedge()->twin()->vertex()->position)/2;
//          e->getVertex()
//          if (!e->isNew == true) {
//           if (!e->halfedge()->vertex()->isNew && !e->halfedge()->twin()->vertex()->isNew)
            if (!e->isNew)
           {
              VertexIter v = mesh.splitEdge(e);
              v->newPosition = e->newPosition;
          }
          // ... we still have a valid reference to the next edge
          e = nextEdge;
      }
    
    // 4. Flip any new edge that connects an old and new vertex.
      e = mesh.edgesBegin();
//      while (e != mesh.edgesEnd()) {
        n = mesh.nEdges();
        for (int i = 0; i < n; i++) {

          // get the next edge NOW!
          EdgeIter nextEdge = e;
          nextEdge++;

          // now, even if splitting the edge deletes it ...
//          if (some condition is met) {
//              mesh.splitEdge(e);
//          }
          VertexIter v_1 = e->halfedge()->vertex();
          VertexIter v_2 = e->halfedge()->twin()->vertex();
          if (e->isNew) {
              if ((v_1->isNew && !v_2->isNew) || (v_2->isNew && !v_1->isNew)) {
                  mesh.flipEdge(e);
              }
          }
//          e->getVertex()

          // ... we still have a valid reference to the next edge
          e = nextEdge;
      }

    // 5. Copy the new vertex positions into final Vertex::position.
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
//          if (v->isNew) {
              v->position = v->newPosition;
//          }
      }
////          cout << v->position << endl; // do something interesting here
//          if(!v->isNew) {
//              v->position = v->newPosition;
//          }  else {
//              v->position = v->halfedge()->edge()->newPosition;
//          }
//      }


  }
}

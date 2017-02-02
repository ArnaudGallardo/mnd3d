
#include "bvh.h"
#include "mesh.h"
#include <iostream>

void BVH::build(const Mesh* pMesh, int targetCellSize, int maxDepth)
{
    // store a pointer to the mesh
    m_pMesh = pMesh;
    // allocate the root node
    m_nodes.resize(1);

    if(m_pMesh->nbFaces() <= targetCellSize) { // only one node
        m_nodes[0].box = pMesh->AABB();
        m_nodes[0].first_face_id = 0;
        m_nodes[0].is_leaf = true;
        m_nodes[0].nb_faces = m_pMesh->nbFaces();
        m_faces.resize(m_pMesh->nbFaces());
        for(int i=0; i<m_pMesh->nbFaces(); ++i)
        {
            m_faces[i] = i;
        }
    }else{
        // reserve space for other nodes to avoid multiple memory reallocations
        m_nodes.reserve( std::min<int>(2<<maxDepth, std::log(m_pMesh->nbFaces()/targetCellSize) ) );

        // compute centroids and initialize the face list
        m_centroids.resize(m_pMesh->nbFaces());
        m_faces.resize(m_pMesh->nbFaces());
        for(int i=0; i<m_pMesh->nbFaces(); ++i)
        {
            m_centroids[i] = (m_pMesh->vertexOfFace(i, 0).position + m_pMesh->vertexOfFace(i, 1).position + m_pMesh->vertexOfFace(i, 2).position)/3.f;
            m_faces[i] = i;
        }

        // recursively build the BVH, starting from the root node and the entire list of faces
        buildNode(0, 0, m_pMesh->nbFaces(), 0, targetCellSize, maxDepth);

    }
}

bool BVH::intersect(const Ray& ray, Hit& hit) const
{
    // compute the intersection with the root node
    float tMin, tMax;
    Normal3f n;
    ::intersect(ray, m_nodes[0].box, tMin, tMax, n);

    // TODO
    // vérifier si on a bien une intersection (en fonction de tMin, tMax, et hit.t()), et si oui appeler intersecNode...
    if (hit.t() > tMin && tMax > 0) {
      this->intersectNode(0,ray,hit);
      return true;
    }

    return false;
}

bool BVH::intersectNode(int nodeId, const Ray& ray, Hit& hit) const
{
    // TODO, deux cas: soit mNodes[nodeId] est une feuille (il faut alors intersecter les triangles du noeud),
    // soit c'est un noeud interne (il faut visiter les fils (ou pas))
    bool found = false;
    if (m_nodes[nodeId].is_leaf) {
      //printf("Leaf : %d\n", nodeId);
      Hit *tmp = new Hit();
      for (int i = m_nodes[nodeId].first_face_id; i < m_nodes[nodeId].first_face_id + m_nodes[nodeId].nb_faces; i++) {
        if (m_pMesh->intersectFace(ray,*tmp,m_faces[i])) {
          if (tmp->t() < hit.t()) {
            hit.setShape(m_pMesh);
            hit.setT(tmp->t());
            hit.setNormal(tmp->normal());
            found = true;
          }
        }
      }
    } else {
      //printf("Node : %d\tChild1 : %d\n", nodeId,m_nodes[nodeId].first_child_id);
      float tMin, tMax;
      Normal3f n;
      int first_child_id = m_nodes[nodeId].first_child_id;
      if (::intersect(ray, m_nodes[first_child_id].box, tMin, tMax, n)) {
        if (hit.t() > tMin) {
          if (this->intersectNode(first_child_id,ray,hit)) {
            found = true;
          }
        }
      }
      if (::intersect(ray, m_nodes[first_child_id+1].box, tMin, tMax, n)) {
        if (hit.t() > tMin) {
          if (this->intersectNode(first_child_id+1,ray,hit)) {
            found = true;
          }
        }
      }
    }

    return found;
}

/** Sorts the faces with respect to their centroid along the dimension \a dim and spliting value \a split_value.
  * \returns the middle index
  */
int BVH::split(int start, int end, int dim, float split_value)
{
    int l(start), r(end-1);
    while(l<r)
    {
        // find the first on the left
        while(l<end && m_centroids[l](dim) < split_value) ++l;
        while(r>=start && m_centroids[r](dim) >= split_value) --r;
        if(l>r) break;
        std::swap(m_centroids[l], m_centroids[r]);
        std::swap(m_faces[l], m_faces[r]);
        ++l;
        --r;
    }
    return m_centroids[l][dim]<split_value ? l+1 : l;
}

void BVH::buildNode(int nodeId, int start, int end, int level, int targetCellSize, int maxDepth)
{
    Node& node = m_nodes[nodeId];

    // étape 1 : calculer la boite englobante des faces indexées de mFaces[start] à mFaces[end]
    // (Utiliser la fonction extend de Eigen::AlignedBox3f et la fonction mpMesh->vertexOfFace(int) pour obtenir les coordonnées des sommets des faces)
    node.box = Eigen::AlignedBox3f(3);
    for (int i = start; i < end; i++) {
      for (int k = 0; k < 3; k++) {
        node.box.extend(m_pMesh->vertexOfFace(m_faces[i],k).position);
      }
    }

    // étape 2 : déterminer si il s'agit d'une feuille (appliquer les critères d'arrêts)
    // Si c'est une feuille, finaliser le noeud et quitter la fonction
    if (level >= maxDepth || end-start <= targetCellSize) {
      node.is_leaf = true;
      node.nb_faces = end-start;
      node.first_face_id = start;
      return;
    } else {
      // Si c'est un noeud interne :
      // étape 3 : calculer l'index de la dimension (x=0, y=1, ou y=2) et la valeur du plan de coupe
      // (on découpe au milieu de la boite selon la plus grande dimension)
      Vector3f tmp = node.box.max() - node.box.min();
      float max = 0;
      int dim = 0;
      for (int i = 0; i < 3; i++) {
        //printf("BoxMax : %f - %f - %f\n", box.max()[0],box.max()[1],box.max()[2]);
        //printf("BoxMin : %f - %f - %f\n", box.min()[0],box.min()[1],box.min()[2]);
        if (tmp[i] > max) {
          max = tmp[i];
          dim = i;
        }
      }

      // étape 4 : appeler la fonction split pour trier (partiellement) les faces
      int middle = this->split(start, end, dim, node.box.min()[dim] + (max / 2.f));

      node.is_leaf = false;
      node.nb_faces = end-start;

      Node node_left;
      Node node_right;

      int size = m_nodes.size();
      node.first_child_id = size; //WUT
      m_nodes.push_back(node_left);
      m_nodes.push_back(node_right);

      //printf("Node : %d\tChild1 : %d\n", nodeId,node.first_child_id);

      // étape 5 : allouer les fils, et les construire en appelant buildNode...
      this->buildNode(size,start,middle,level+1,targetCellSize,maxDepth);
      this->buildNode(size+1,middle,end,level+1,targetCellSize,maxDepth);
    }
}

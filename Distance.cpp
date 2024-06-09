#include <cmath>
#include <iostream>
#include <vector>
using namespace std;
struct Point
{
   float x;
   float y;
   float z;
};

struct Vector
{
   float i;
   float j;
   float k;
};

double MagSq(Vector u)
{
   return (u.i * u.i + u.j * u.j + u.k * u.k);
}

double Mag(Vector u)
{
   double rMagSq = MagSq(u);
   return sqrt(rMagSq);
}

Vector Normalized(Vector r)
{
   r.i = r.i / Mag(r); 
   r.j = r.j / Mag(r); 
   r.k = r.k / Mag(r);
   return r;
}


double Dot(Vector u, Vector v)
{
   return (u.i * v.i + u.j * v.j + u.k * v.k);
}

Vector Vectorx(Vector u, double K)
{
   u.i = u.i * K; u.j = u.j * K; u.k = u.k * K;
   return u;
}

Point pointX(Point p, double K)
{
   p.x = p.x * K; p.y = p.y * K; p.z = p.z * K;
   return p;
}

Point sumOfPoints(Point p, Point q)
{
   Point r;
   r.x = p.x + q.x;
   r.y = p.y + q.y;
   r.z = p.z + q.z;
   return r;
}

Point differenceOfPoints(Point p, Point q)
{
   Point r;
   r.x = p.x - q.x;
   r.y = p.y - q.y;
   r.z = p.z - p.z;
   return r;
}

Point vectorToPoint(Vector u)
{
   Point p;
   p.x = u.i;
   p.y = u.j;
   p.z = u.k;
   return p;
}

Vector pointToVector(Point p)
{
   Vector u;
   u.i = p.x;
   u.j = p.y;
   u.k = p.z;
   return u;
}

double distancePointToPoint(Point p, Point q)
{
   double distance =  sqrt((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y) + (p.z - q.z) * (p.z - q.z));
   return distance;
}


double lineStartToPlaneRelative( 
          Point line_start_point, 
          Point ray_origin, 
          Vector ray_direction)
{
   Vector line_start_direction;
   line_start_direction.i = line_start_point.x - ray_origin.x;
   line_start_direction.j = ray_origin.y - ray_origin.y;
   line_start_direction.k = ray_origin.z - ray_origin.z;
   double relativeStartPointDistance = Dot(line_start_direction, Normalized(ray_direction));
   return relativeStartPointDistance;
}


double lineEndToPlaneRelative(
   Point line_end_point,
   Point ray_origin,
   Vector ray_direction)
{
   Vector line_end_direction;
   line_end_direction.i = line_end_point.x - ray_origin.x;
   line_end_direction.j = ray_origin.y - ray_origin.y;
   line_end_direction.k = ray_origin.z - ray_origin.z;
   double relativeEndPointDistance = Dot(line_end_direction, Normalized(ray_direction));
   return relativeEndPointDistance;
}


double lineToLineSegment(
   Point line_start_point,
   Point line_end_point,
   Point ray_origin,
   Vector ray_direction)
{
   double distanceStart = lineStartToPlaneRelative(line_start_point, ray_origin, ray_direction);
   Point pointStartOnRay = sumOfPoints(ray_origin, vectorToPoint(Vectorx(Normalized(ray_direction), distanceStart)));
   double distanceStartPointToRay = distancePointToPoint(line_start_point, pointStartOnRay);

   double distanceEnd = lineEndToPlaneRelative(line_end_point, ray_origin, ray_direction);
   Point pointEndOnRay = sumOfPoints(ray_origin, vectorToPoint(Vectorx(Normalized(ray_direction), distanceEnd)));
   double distanceEndPointToRay = distancePointToPoint(line_end_point, pointEndOnRay);
   double dist;
   if (distanceStartPointToRay < distanceEndPointToRay)
   {
      dist = distanceStartPointToRay;
   }
   else
   {
      dist = distanceEndPointToRay;
   }
   return dist;
}


double pointToLineSegment(
   Point line_start_point,
   Point line_end_point,
   Point ray_origin)
{
   Vector lineDirection = pointToVector(differenceOfPoints(line_end_point, line_start_point));
   Vector rayOriginStartDirection = pointToVector(differenceOfPoints(ray_origin, line_start_point));
   Vector rayOriginEndDirection = pointToVector(differenceOfPoints(ray_origin, line_end_point));
   double projectedDistanceStart = Dot(rayOriginStartDirection, Normalized(lineDirection));
   double projectedDistanceEnd = Dot(rayOriginEndDirection, Normalized(lineDirection));

   //double rayOriginRelativeDistanceStart = lineStartToPlaneRelative(
   //   ray_origin,
   //   line_start_point,
   //   lineDirection);

   //double rayOriginRelativeDistanceEnd = lineStartToPlaneRelative(
   //   ray_origin,
   //   line_end_point,
   //   lineDirection);

   //double lineStartToPlaneRelative(
   //   Point line_start_point,
   //   Point ray_origin,
   //   Vector ray_direction);

   double dist = 0;
   if (projectedDistanceStart < 0 && projectedDistanceEnd < 0)
   {
     // *pClosestPointOnRay = ray_origin;
      //*pClosestPointOnLineSegment = line_start_point;
      dist = distancePointToPoint(line_start_point, ray_origin);
   }
   else if (projectedDistanceStart > 0 && projectedDistanceEnd > 0)
   {
      //*pClosestPointOnRay = ray_origin;
      //*pClosestPointOnLineSegment = line_end_point;
      dist = distancePointToPoint(line_end_point, ray_origin);
   }
   else
   {

      if (projectedDistanceStart < projectedDistanceEnd)
      {
         //*pClosestPointOnRay = ray_origin;
         Point pointOnLine = sumOfPoints(line_start_point, vectorToPoint(Vectorx(Normalized(lineDirection), projectedDistanceStart)));
         dist = distancePointToPoint(pointOnLine, ray_origin);
      }

      else
      {
         //*pClosestPointOnRay = ray_origin;
         Point pointOnLine = sumOfPoints(line_end_point, vectorToPoint(Vectorx(Normalized(lineDirection), projectedDistanceEnd)));
         dist = distancePointToPoint(pointOnLine, ray_origin);
      }
   }
   return dist;
}


Point InterpolatePointOnLine(
   Point pt1,
   Point pt2,
   double param1,
   double param2,
   double paramTarget)
{
   double q = (paramTarget - param1)/(param2 - param1);
   Point result;
   result.x = pt2.x + (pt1.x - pt2.x) * q;
   result.y = pt2.y + (pt1.y - pt2.y) * q;
   result.z = pt2.z + (pt1.z - pt2.z) * q;
   return result;
}


double LineSegmentToRay(
   Point line_start_point,
   Point line_end_point,
   Point ray_origin,
   Vector ray_direction)
{
   /*double ray_length = MagSq(ray_direction);
   if (ray_length == 0)
   {
     return;
   }*/

   double distStart = lineStartToPlaneRelative(line_start_point, ray_origin, ray_direction);
   double distEnd = lineEndToPlaneRelative(line_end_point, ray_origin, ray_direction);

   Point interpolatePoint = InterpolatePointOnLine(
      line_start_point,
      line_end_point,
      distStart,
      distEnd,
      0.0);

   double dist;
   if (distStart > 0 && distEnd > 0)
   {
      dist = lineToLineSegment(
         line_start_point,
         line_end_point,
         ray_origin,
         ray_direction);
   }
   else if (distStart < 0 && distEnd < 0)
   {
      dist =  pointToLineSegment(
         line_start_point,
         line_end_point,
         ray_origin);
   }
   else if(distStart < 0 && distEnd > 0)
   {
      double dist1 =  pointToLineSegment(
         line_start_point,
         interpolatePoint,
         ray_origin);

      double dist2 =  lineToLineSegment(
         interpolatePoint,
         line_end_point,
         ray_origin,
         ray_direction);

      if (dist1 < dist2)
      {
         dist = dist1;
      }
      else
      {
         dist = dist2;
      }
   }
   else
   {
       double dist1 =  pointToLineSegment(
         line_end_point,
         interpolatePoint,
         ray_origin);

      double dist2 = lineToLineSegment(
         interpolatePoint,
         line_start_point,
         ray_origin,
         ray_direction);

      if (dist1 < dist2)
      {
         dist = dist1;
      }
      else
      {
         dist = dist2;
      }
   }
   return dist;
}


double NearestLineSegToRay(Point ray_origin,
   Vector ray_direction)
{
   int numLineSegments;
   std::cout << "Enter the number of Line segments: ";
   std::cin >> numLineSegments;

   std::vector<double> nearestDistances(numLineSegments, std::numeric_limits<double>::max()); // Initialize with infinity

   for (int lineSegmentIndex = 0; lineSegmentIndex < numLineSegments; lineSegmentIndex++)
   {
      Point line_start_point;
      Point line_end_point;
      ray_origin;
      ray_direction;

      std::cout << "Enter the coordinates of the start point of "
         << lineSegmentIndex << " line segment: ";
      std::cin >> line_start_point.x >> line_start_point.y >> line_start_point.z;

      std::cout << "Enter the coordinates of the end point of "
         << lineSegmentIndex << " line segment: ";
      std::cin >> line_end_point.x >> line_end_point.y >> line_end_point.z;

      nearestDistances[lineSegmentIndex] = LineSegmentToRay(line_start_point,
         line_end_point,
         ray_origin,
         ray_direction);
   }

   double minDistance = nearestDistances[0];
   for (int i = 1; i < numLineSegments; i++) {
      minDistance = std::min(minDistance, nearestDistances[i]);
   }
   return minDistance;
}

enum EAxis
{
   EAxis_X = 0,
   EAxis_Y,
   EAxis_Z
};

class CKDTreeNode
{
   public:
      CKDTreeNode* pParent;

   union
   {
      struct
      {
         CKDTreeNode* pLeft;
         CKDTreeNode* pRight;
      };

      CKDTreeNode* child[2];
   };

   EAxis splitAxis;
   double splitValue;

   Point minPt;
   Point maxPt;

   CKDTreeNode();

   bool IsLeaf() const;

};


CKDTreeNode::CKDTreeNode() :
   pParent(nullptr),
   pLeft(nullptr),
   pRight(nullptr),
   splitAxis(EAxis_X),
   splitValue(0.0) {}


struct SQIndex
{
public:
   size_t collectionIndex;
   size_t entityIndex;
   SQIndex();
   SQIndex(size_t collectionIndex, size_t entityIndex);
};

bool CKDTreeNode::IsLeaf() const
{
   return pLeft == nullptr && pRight == nullptr;
}

class CKDTreeLeafNode : public CKDTreeNode
{
public:
   std::vector<SQIndex> storedIndices;
   CKDTreeLeafNode();
};

class CKDTree
{
public:
   CKDTreeNode* pRootNode;
   CKDTreeNode nodesInternal;
   CKDTreeLeafNode nodesLeaf;
   Point minPt;
   Point maxPt;
   CKDTree();
   ~CKDTree();
};

CKDTree::CKDTree() : pRootNode(nullptr) {}
CKDTree::~CKDTree() {}


class CKDTreeTraverser_RayRadial
{

public:
   const CKDTree& m_tree;
   Point m_rayOrigin;
   Vector m_rayDirectionNormalized;
   std::vector<const CKDTreeNode*> m_traversalStack;

   CKDTreeTraverser_RayRadial(
      const CKDTree& tree,
      Point ray_origin,
      Vector rayDirectionNormalized);

   const CKDTreeLeafNode* GetNext(double maxDistance);
   void Reset();

private:

   bool RadialDistanceBoundingBox(
   Point ray_origin,
      Vector rayDirectionNormalized,
      double* pMaxDistance,
      Point minPt,
      Point maxPt);
};


const CKDTreeLeafNode* CKDTreeTraverser_RayRadial::GetNext(double maxDistance)
{
   while (!m_traversalStack.empty())
   {
      const CKDTreeNode* pCurrentNode = m_traversalStack.back();
      m_traversalStack.pop_back();

      if (RadialDistanceBoundingBox(
         m_rayOrigin, 
         m_rayDirectionNormalized, 
         &maxDistance, 
         pCurrentNode->minPt, 
         pCurrentNode->maxPt) == false) continue;

      if (pCurrentNode->IsLeaf())
      {
         return static_cast<const CKDTreeLeafNode*>(pCurrentNode);
      }

      if (pCurrentNode->pLeft == nullptr)
      {
         m_traversalStack.push_back(pCurrentNode->pRight);
      }
      else if (pCurrentNode->pRight == nullptr)
      {
         m_traversalStack.push_back(pCurrentNode->pLeft);
      }
      else
      {
         double leftNodeDist = RadialDistanceBoundingBox(
            m_rayOrigin,
            m_rayDirectionNormalized,
            &maxDistance,
            pCurrentNode->pLeft->minPt,
            pCurrentNode->pLeft->maxPt);

         double rightNodeDist = RadialDistanceBoundingBox(
            m_rayOrigin,
            m_rayDirectionNormalized,
            &maxDistance,
            pCurrentNode->pRight->minPt,
            pCurrentNode->pRight->maxPt);

         if (leftNodeDist < rightNodeDist)
         {
            m_traversalStack.push_back(pCurrentNode->pLeft);
            m_traversalStack.push_back(pCurrentNode->pRight);
         }
      }
   }
   return nullptr;
}

void CKDTreeTraverser_RayRadial::Reset()
{
   m_traversalStack.clear();

   if (m_tree.pRootNode == nullptr) return;

   m_traversalStack.reserve(100);
   m_traversalStack.push_back(m_tree.pRootNode);
}


bool CKDTreeTraverser_RayRadial::RadialDistanceBoundingBox(
   Point rayOrigin,
   Vector rayDirectionNormalized,
   double* pMaxDistance,
   Point minPt,
   Point maxPt)
{
   Point sphereCentre = pointX(sumOfPoints(maxPt, minPt), 0.5);
   double sphereRadius = distancePointToPoint(sphereCentre, maxPt);
   double sphereRadiusSq = sphereRadius * sphereRadius;
   Vector lineStartToRayOrigin = pointToVector(differenceOfPoints(sphereCentre, rayOrigin));
   double distBoxCentreToRayPlane = Dot(rayDirectionNormalized, lineStartToRayOrigin);
   double distBoxCentreToRayPlaneSq = distBoxCentreToRayPlane * distBoxCentreToRayPlane;
   Point closestPointOnRay = sumOfPoints(rayOrigin , vectorToPoint(Vectorx(rayDirectionNormalized, distBoxCentreToRayPlane)));
   double distCentreToRaySq;

   if (distBoxCentreToRayPlane < 0.0)
   {
      if (distBoxCentreToRayPlane < sphereRadius)
      {
         sphereRadiusSq -= distBoxCentreToRayPlaneSq;
         distCentreToRaySq = distancePointToPoint(closestPointOnRay, sphereCentre);
         if (distCentreToRaySq < sphereRadiusSq)
         {
            distCentreToRaySq = 0.0;
         }
      }
      else
      {
         distCentreToRaySq = distancePointToPoint(rayOrigin, sphereCentre);
      }
   }
   else
   {
      distCentreToRaySq = distancePointToPoint(closestPointOnRay, sphereCentre);
      if (distCentreToRaySq < sphereRadiusSq)
      {
         distCentreToRaySq = 0;
      }
   }

   if (distCentreToRaySq > *pMaxDistance)
   {
      return false;
   }
   *pMaxDistance = distCentreToRaySq;
   return true;
}


double NearestLineSegToRayUsingKDTree(Point ray_origin, Vector ray_direction)
{
   // Initialize the traverser with the ray
   CKDTreeTraverser_RayRadial traverser(CKDTree, ray_origin, ray_direction);

   // Loop through each leaf node untill that may contains the nearest line
   // segment are exhausted

   CKDTreeLeafNode* pCurrentCandidate = nullptr;

   while ((pCurrentCandidate = traverser.GetNext(closestDistance)) != nullptr)
   {
      for (size_t i = 0; i < pCurrentCandidate->storedIndices.Count(); i++)
      {

      }
   }
}

int main() 
{
   Point ray_origin;
   std::cout << "Enter the coordinates of the ray origin: ";
   std::cin >> ray_origin.x >> ray_origin.y >> ray_origin.z;
   Vector ray_direction;
   std::cout << "Enter the ray direction: ";
   std::cin >> ray_direction.i >> ray_direction.j >> ray_direction.k;
   double minDistance = NearestLineSegToRay(ray_origin, ray_direction);
   std::cout << "Minimum distance: " << minDistance << std::endl;
}

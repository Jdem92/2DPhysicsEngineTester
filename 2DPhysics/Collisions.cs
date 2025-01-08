using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace _2DPhysics
{
    public static class Collisions
    {

        //this will return the distance between a line segment and a point
        //it will also return the actual contact point (closest point in space)
        //https://www.youtube.com/watch?v=egmZJU-1zPU&t=0s
        public static void PointSegmentDistance(_2DVector p, _2DVector a, _2DVector b, out float distanceSquared, out _2DVector closestPoint)
        {
            _2DVector ab = b - a;
            _2DVector ap = p - a;

            float proj = _2DMath.Dot(ap, ab);
            float abLengthSquared = _2DMath.LengthSquared(ab);
            float d = proj / abLengthSquared;

            if (d <= 0f)
            {
                closestPoint = a;
            } 
            else if (d >= 1f)
            {
                closestPoint = b;
            }
            else
            {
                closestPoint = a + ab * d;
            }

            distanceSquared = _2DMath.DistanceSquared(p, closestPoint);
        }

        //Axis aligned bounding box [ ] <--> [ ] comparing edges of objects
        public static bool IntersectAABB(_2DAABB a, _2DAABB b)
        {
            if (a.Max.X <= b.Min.X || b.Max.X <= a.Min.X ||
                a.Max.Y <= b.Min.Y || b.Max.Y <= a.Min.Y)
            {
                return false;
            }
            return true;
        }

        public static void FindContactPoint(_2DBody bodyA, _2DBody bodyB, out _2DVector contactPoint1, out _2DVector contactPoint2, out int contactCount)
        {
            contactPoint1 = _2DVector.Zero;
            contactPoint2 = _2DVector.Zero;
            contactCount = 0;

            ShapeType shapeTypeA = bodyA.shapeType;
            ShapeType shapeTypeB = bodyB.shapeType;

            if (shapeTypeA == ShapeType.Box)
            {
                if (shapeTypeB == ShapeType.Box)
                {
                    Collisions.FindContactPoint(bodyA.GetTransformedVertices(), bodyB.GetTransformedVertices(), out contactPoint1, out contactPoint2, out contactCount);
                }
                else if (shapeTypeB == ShapeType.Circle)
                {
                    Collisions.FindContactPoint(bodyB.Position, bodyB.Radius, bodyA.Position, bodyA.GetTransformedVertices(), out contactPoint1);
                    contactCount = 1;
                }
            }
            else if (shapeTypeA == ShapeType.Circle)
            {
                if (shapeTypeB == ShapeType.Box)
                {
                    Collisions.FindContactPoint(bodyA.Position, bodyA.Radius, bodyB.Position, bodyB.GetTransformedVertices(), out contactPoint1);
                    contactCount = 1;
                }
                else if (shapeTypeB == ShapeType.Circle)
                {
                    Collisions.FindContactPoint(bodyA.Position, bodyA.Radius, bodyB.Position, out contactPoint1);
                    contactCount = 1;
                }
            }
        }

        //
        private static void FindContactPoint(_2DVector[] verticesA, _2DVector[] verticesB, out _2DVector contact1, out _2DVector contact2, out int contactCount)
        {
            contact1 = _2DVector.Zero;
            contact2 = _2DVector.Zero;
            contactCount = 0;

            float minDistanceSquared = float.MaxValue;

            for (int i = 0; i < verticesA.Count(); i++)
            {
                _2DVector p = verticesA[i];

                for (int j = 0; j < verticesB.Count(); j++)
                {
                    //get edges
                    _2DVector va = verticesB[j];
                    _2DVector vb = verticesB[(j + 1) % verticesB.Length];

                    Collisions.PointSegmentDistance(p, va, vb, out float distanceSquared, out _2DVector contactPoint);

                    if (_2DMath.NearlyEqual(distanceSquared, minDistanceSquared)) //distance is the same, comparing floating point values
                    {
                        if (!_2DMath.NearlyEqual(contactPoint, contact1))
                        {
                            contact2 = contactPoint;
                            contactCount = 2;
                        }
                    }
                    else if (distanceSquared < minDistanceSquared)
                    {
                        minDistanceSquared = distanceSquared;
                        contactCount = 1;
                        contact1 = contactPoint;
                    }
                }
            }

            for (int i = 0; i < verticesB.Count(); i++)
            {
                _2DVector p = verticesB[i];

                for (int j = 0; j < verticesA.Count(); j++)
                {
                    //get edges
                    _2DVector va = verticesA[j];
                    _2DVector vb = verticesA[(j + 1) % verticesA.Length];

                    Collisions.PointSegmentDistance(p, va, vb, out float distanceSquared, out _2DVector contactPoint);

                    //
                    if (_2DMath.NearlyEqual(distanceSquared, minDistanceSquared)) //distance is the same, comparing floating point values
                    {
                        if (!_2DMath.NearlyEqual(contactPoint, contact1))
                        {
                            contact2 = contactPoint;
                            contactCount = 2;
                        }

                    }
                    else if (distanceSquared < minDistanceSquared)
                    {
                        minDistanceSquared = distanceSquared;
                        contactCount = 1;
                        contact1 = contactPoint;
                    }
                }
            }
        }

        
        private static void FindContactPoint(_2DVector circleCenter, float circleRadius, _2DVector polygonCenter, _2DVector[] polygonVertices, out _2DVector cp)
        {
            cp = _2DVector.Zero;
            float minDistanceSquared = float.MaxValue; 

            //loop through every edge of the polygon, loop through vertices
            for (int i = 0; i < polygonVertices.Count(); i++)
            {
                _2DVector va = polygonVertices[i]; //point va for current vertex
                _2DVector vb = polygonVertices[(i + 1) % polygonVertices.Length]; //loop back around if we get to the end of the vertice

                Collisions.PointSegmentDistance(circleCenter, va, vb, out float distanceSquared, out _2DVector contact);

                if (distanceSquared < minDistanceSquared)
                {
                    minDistanceSquared = distanceSquared;
                    cp = contact;
                }
            }
        }

        //circle to circle collision (1 contact point)
        private static void FindContactPoint(_2DVector centerA, float radiusA, _2DVector centerB, out _2DVector contactPoint)
        {
            //we need a vector pointing from the 1st circle to the 2nd circle
            _2DVector ab = centerB - centerA;

            _2DVector directionVector = _2DMath.Normalize(ab);
            contactPoint = centerA + directionVector * radiusA;
        }

        //
        public static bool Collide(_2DBody bodyA, _2DBody bodyB, out _2DVector normal, out float depth)
        {
            normal = _2DVector.Zero;
            depth = 0f;

            ShapeType shapeTypeA = bodyA.shapeType;
            ShapeType shapeTypeB = bodyB.shapeType;

            if (shapeTypeA == ShapeType.Box)
            {
                if (shapeTypeB == ShapeType.Box)
                {
                    return Collisions.IntersectPolygons(bodyA.Position, bodyA.GetTransformedVertices(),
                                                        bodyB.Position, bodyB.GetTransformedVertices(),
                                                        out normal, out depth);
                }
                else if (shapeTypeB == ShapeType.Circle)
                {
                    bool result = Collisions.IntersectCirclePolygon(bodyB.Position, bodyB.Radius,
                                                                    bodyA.Position, bodyA.GetTransformedVertices(),
                                                                    out normal, out depth);

                    //reverse normal here bc normal would be incorrct (to push bodyA away from bodyB)
                    normal = -normal;
                    return result;
                }
            }
            else if (shapeTypeA == ShapeType.Circle)
            {
                if (shapeTypeB == ShapeType.Box)
                {
                    return Collisions.IntersectCirclePolygon(bodyA.Position, bodyA.Radius,
                                                                bodyA.Position, bodyB.GetTransformedVertices(),
                                                                out normal, out depth);
                }
                else if (shapeTypeB == ShapeType.Circle)
                {
                    return Collisions.IntersectCircles(bodyA.Position, bodyA.Radius,
                                                        bodyB.Position, bodyB.Radius,
                                                        out normal, out depth);
                }
            }

            //if no collision then just return false
            return false;

        }

        //the plan is to make all the collisions using the "seperating axis theroem"
        public static bool IntersectCirclePolygon(_2DVector circleCenter, float circleRadius,
                                                    _2DVector polygonCenter, _2DVector[] vertices, 
                                                    out _2DVector normal, out float depth)
        {
            normal = _2DVector.Zero;
            depth = float.MaxValue; //for comparison operation

            _2DVector axis = _2DVector.Zero;
            float axisDepth = 0f;
            float minA, maxA, minB, maxB;

            for (int i = 0; i < vertices.Count(); i++)
            {
                _2DVector va = vertices[i];
                _2DVector vb = vertices[(i + 1) % vertices.Length];

                _2DVector edge = vb - va;
                //axis to test for seperation
                axis = new _2DVector(-edge.Y, edge.X);
                axis = _2DMath.Normalize(axis); //normalize the axis 

                Collisions.ProjectVertices(vertices, axis, out minA, out maxA);
                Collisions.ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);

                //compare the min and max
                if (minA >= maxB || minB >= maxA)
                {
                    //these are seperated
                    return false;
                }

                //store the minimum depth required to resolve the collision
                axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }
            }

            int cpIndex = Collisions.FindClosestPointOnPolygon(circleCenter, vertices);
            _2DVector cp = vertices[cpIndex];

            axis = cp - circleCenter;
            axis = _2DMath.Normalize(axis); //normalize the axis 

            Collisions.ProjectVertices(vertices, axis, out minA, out maxA);
            Collisions.ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);

            //compare the min and max
            if (minA >= maxB || minB >= maxA)
            {
                //these are seperated
                return false;
            }

            //store the minimum depth required to resolve the collision
            axisDepth = MathF.Min(maxB - minA, maxA - minB);

            if (axisDepth < depth)
            {
                depth = axisDepth;
                normal = axis;
            }

            _2DVector direction = polygonCenter - circleCenter;

            //use dot product
            if (_2DMath.Dot(direction, normal) < 0f)
            {
                normal = -normal;
            }

            return true;
        }


        private static int FindClosestPointOnPolygon(_2DVector circleCenter, _2DVector[] vertices)
        {
            int result = -1;
            float minDistance = float.MaxValue;

            for (int i = 0; i < vertices.Count(); i++)
            {
                _2DVector v = vertices[i];
                float distance = _2DMath.Distance(v, circleCenter);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    result = i;
                }
            }

            return result;
        }


        private static void ProjectCircle(_2DVector center, float radius, _2DVector axis, out float min, out float max)
        {
            _2DVector direction = _2DMath.Normalize(axis);
            _2DVector directionAndRadius = direction * radius;

            _2DVector p1 = center + directionAndRadius;
            _2DVector p2 = center - directionAndRadius;

            min = _2DMath.Dot(p1, axis);
            max = _2DMath.Dot(p2, axis);

            if (min > max)
            {
                // swap min and max values
                float t = min;
                min = max;
                max = t;
            }
        }

        //Are they or are they not intersecting
        public static bool IntersectPolygons(_2DVector centerA, _2DVector[] verticesA, 
                                                _2DVector centerB, _2DVector[] verticesB, 
                                                out _2DVector normal, out float depth)
        {
            normal = _2DVector.Zero;
            depth = float.MaxValue; //for comparison operation

            for (int i = 0; i < verticesA.Count(); i++)
            {
                _2DVector va = verticesA[i];
                _2DVector vb = verticesA[(i + 1) % verticesA.Length];

                _2DVector edge = vb - va;
                //axis to test for seperation
                _2DVector axis = new _2DVector(-edge.Y, edge.X);
                axis = _2DMath.Normalize(axis); //normalize axis

                Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
                Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);

                //compare the min and max
                if (minA >= maxB || minB >= maxA)
                {
                    //these are seperated
                    return false;
                }

                //store the minimum depth required to resolve the collision
                float axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }
            }

            for (int i = 0; i < verticesB.Count(); i++)
            {
                _2DVector va = verticesB[i];
                _2DVector vb = verticesB[(i + 1) % verticesB.Length];

                _2DVector edge = vb - va;
                //axis to test for seperation
                _2DVector axis = new _2DVector(-edge.Y, edge.X);
                axis = _2DMath.Normalize(axis); //normalize axis

                Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
                Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);

                //compare the min and max
                if (minA >= maxB || minB >= maxA)
                {
                    //these are seperated
                    return false;
                }

                //store the minimum depth required to resolve the collision
                float axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }
            }

            _2DVector direction = centerB - centerA;

            //use dot product
            if (_2DMath.Dot(direction, normal) < 0f)
            {
                normal = -normal;
            }

            //there is a collision occuring
            return true;
        }


        private static void ProjectVertices(_2DVector[] verticies, _2DVector axis, out float min, out float max)
        {
            min = float.MaxValue;
            max = float.MinValue;

            for (int i = 0; i < verticies.Count(); i++) 
            { 
                _2DVector v = verticies[i];
                float projection = _2DMath.Dot(v, axis);

                if (projection < min)
                {
                    min = projection;
                }
                if (projection > max)
                {
                    max = projection;
                }
            }
        }

        //circles are good bc they are fast and efficient
        public static bool IntersectCircles(_2DVector centerA, float radiusA, _2DVector centerB, float radiusB, out _2DVector normal, out float depth)
        {
            //first thing to do is to determine how far apart the circles are from eachother
            normal = _2DVector.Zero;
            depth = 0f;

            float distance = _2DMath.Distance(centerA, centerB);
            float radii = radiusA + radiusB;

            if (distance >= radii) 
            {
                return false;
            }

            normal = _2DMath.Normalize(centerB - centerA);
            depth = radii - distance;

            return true;
        }

        #region OldCode
        //find center of objects
        //private static _2DVector FindArithmeticMean(_2DVector[] verticies)
        //{
        //    float sumX = 0f;
        //    float sumY = 0f;

        //    for (int i = 0; i < verticies.Count(); i++)
        //    {
        //        _2DVector v = verticies[i];
        //        sumX += v.X;
        //        sumY += v.Y;    
        //    }

        //    return new _2DVector(sumX / (float)verticies.Length, sumY / (float)verticies.Length);
        //}
        #endregion

    }
}

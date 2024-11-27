﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace _2DPhysics
{
    public static class Collisions
    {
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

        public static bool IntersectCirclePolygon(_2DVector circleCenter, float circleRadius, _2DVector[] vertices, out _2DVector normal, out float depth)
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

            //Ensure polygon collision resolution
            _2DVector polygonCenter = Collisions.FindArithmeticMean(vertices);

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

        public static bool IntersectPolygons(_2DVector[] verticesA, _2DVector[] verticesB, out _2DVector normal, out float depth)
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

            //Ensure polygon collision resolution
            _2DVector centerA = Collisions.FindArithmeticMean(verticesA);
            _2DVector centerB = Collisions.FindArithmeticMean(verticesB);

            _2DVector direction = centerB - centerA;

            //use dot product
            if (_2DMath.Dot(direction, normal) < 0f)
            {
                normal = -normal;
            }

            //there is a collision occuring
            return true;
        }

        //find center of objects
        private static _2DVector FindArithmeticMean(_2DVector[] verticies)
        {
            float sumX = 0f;
            float sumY = 0f;

            for (int i = 0; i < verticies.Count(); i++)
            {
                _2DVector v = verticies[i];
                sumX += v.X;
                sumY += v.Y;    
            }

            return new _2DVector(sumX / (float)verticies.Length, sumY / (float)verticies.Length);
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


    }
}
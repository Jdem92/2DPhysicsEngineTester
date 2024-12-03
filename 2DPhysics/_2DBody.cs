using System;
using System.Collections.Generic;
using System.Linq;
using System.Linq.Expressions;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace _2DPhysics
{
    public enum ShapeType
    {
        Circle = 0,
        Box = 1
    }

    public sealed class _2DBody
    {
        private _2DVector position;
        private _2DVector linearVelocity;
        private float rotation;
        private float rotationVelocity;

        private _2DVector force; //

        public readonly float Density;
        public readonly float Mass;
        public readonly float InvMass;

        public readonly float Restitution;
        public readonly float Area;

        public readonly bool IsStatic;

        public readonly float Radius;
        public readonly float Width;
        public readonly float Height;

        //These are the verticies around the origin (boxes, polygon)
        private readonly _2DVector[] verticies;

        //Stores the indicies of the triangles for the box body verticies
        public readonly int[] triangles;

        //These are the transformed verticies
        private readonly _2DVector[] transformedVerticies;

        //bool to tell if transformed verticies need to be updated
        private bool transformUpdateRequied;

        public readonly ShapeType shapeType;

        public _2DVector Position
        {
            get { return position; }
        }

        public _2DVector LinearVelocity
        {
            get { return this.linearVelocity; }
            internal set { this.linearVelocity = value; }
        }

        //CONSTRUCTOR
        private _2DBody(_2DVector position,
                        float density,
                        float mass,
                        float restitution,
                        float area,
                        bool isStatic,
                        float radius,
                        float width,
                        float height,
                        ShapeType shapeType)
        {
            this.position = position;
            this.linearVelocity = _2DVector.Zero;
            this.rotation = 0f;
            this.rotationVelocity = 0f;

            this.force = _2DVector.Zero;

            this.Density = density;
            this.Mass = mass;
            this.Restitution = restitution;
            this.Area = area;

            this.IsStatic = isStatic;
            this.Radius = radius;
            this.Width = width;
            this.Height = height;
            this.shapeType = shapeType;

            if (!this.IsStatic)
            {
                this.InvMass = 1f / this.Mass;
            }
            else
            {
                this.InvMass = 0f;
            }

            if (this.shapeType == ShapeType.Box)
            {
                this.verticies = CreateBoxVerticies(this.Width, this.Height);
                this.triangles = CreateBoxTriangles();
                this.transformedVerticies = new _2DVector[this.verticies.Length];
            }
            else
            {
                this.verticies = null;
                this.transformedVerticies = null;
                this.triangles = null;
            }

            this.transformUpdateRequied = true;
        }

        private static _2DVector[] CreateBoxVerticies(float width, float height)
        {
            float left = -width / 2f;
            float right = left + width;
            float bottom = -height / 2f;
            float top = bottom + height;

            _2DVector[] verticies = new _2DVector[4];
            verticies[0] = new _2DVector(left, top);
            verticies[1] = new _2DVector(right, top);
            verticies[2] = new _2DVector(right, bottom);
            verticies[3] = new _2DVector(left, bottom);

            return verticies;
        }

        //triangulate a box, it's super trivial apparently
        private static int[] CreateBoxTriangles()
        {
            int[] triangles = new int[6]; //will always be 6 of them

            triangles[0] = 0; //start at top left
            triangles[1] = 1; //top right
            triangles[2] = 2; //bottom right
            triangles[3] = 0; //top left
            triangles[4] = 2; //index #2 which is bottom right
            triangles[5] = 3; //bottom left

            return triangles;
        }

        public _2DVector[] GetTransformedVerticies()
        {
            //first check array
            if (this.transformUpdateRequied)
            {
                _2DTransform transform = new _2DTransform(this.position, this.rotation);

                for (int i = 0; i < this.verticies.Count(); i++)
                {
                    _2DVector v = this.verticies[i];
                    this.transformedVerticies[i] = _2DVector.Transform(v, transform);
                }
            }

            this.transformUpdateRequied = false; //reset the flag
            return this.transformedVerticies;
        }

        internal void Step(float time, _2DVector gravity, int iterations)
        {
            if (this.IsStatic)
            {
                return;
            }

            time /= (float)iterations; //amount of time we want to move on this iteration (this means more accurate movement)

            // force = mass * acceleration
            // acc = force / mass

            //_2DVector acceleration = this.force / this.Mass;
            //this.linearVelocity += acceleration * time;

            this.linearVelocity += gravity * time;

            this.position += this.linearVelocity * time;
            this.rotation += this.rotationVelocity * time;

            this.force = _2DVector.Zero;
            this.transformUpdateRequied = true;
        }

        public void Move(_2DVector amount)
        {
            this.position += amount;
            this.transformUpdateRequied = true;
        }

        public void MoveTo(_2DVector position)
        {
            this.position = position;
            this.transformUpdateRequied = true;
        }

        //function that allows us to rotate
        public void Rotate(float amount)
        {
            this.rotation += amount;
            this.transformUpdateRequied = true;
        }
        
        public void AddForce(_2DVector amount)
        {
            this.force = amount;
        }

        public static bool CreateCircleBody(float radius, _2DVector position, float density, bool isStatic, float restitution, out _2DBody body, out string errorMessage)
        {
            body = null;
            errorMessage = string.Empty;

            //first thing we do is calculate area
            float area = (float)(radius * radius * Math.PI);

            if (area < _2DWorld.MinBodySize)
            {
                errorMessage = $"Circle radius is too small. Min circle area is {_2DWorld.MinBodySize}";
                return false;
            }

            if (area > _2DWorld.MaxBodySize)
            {
                errorMessage = $"Circle radius is too large. Max circle area is {_2DWorld.MaxBodySize}";
                return false;
            }

            if (density < _2DWorld.MinDensity)
            {
                errorMessage = $"Density is too small. Min density area is {_2DWorld.MinDensity}";
                return false;
            }

            if (density > _2DWorld.MaxDensity)
            {
                errorMessage = $"Density is too large. Max density area is {_2DWorld.MaxDensity}";
                return false;
            }

            //force the restitution to be between 0 and 1
            restitution = _2DMath.Clamp(restitution, 0f, 1f);

            //calculate mass of circle (cylinder with a depth/height of 1)
            // mass = area * depth * density
            float mass = area * density;

            body = new _2DBody(position, density, mass, restitution, area, isStatic, radius, 0f, 0f, ShapeType.Circle);
            return true;
        }

        public static bool CreateBoxBody(float width, float height, _2DVector position, float density, bool isStatic, float restitution, out _2DBody body, out string errorMessage)
        {
            body = null;
            errorMessage = string.Empty;

            //first thing we do is calculate area
            float area = width * height;

            if (area < _2DWorld.MinBodySize)
            {
                errorMessage = $"Area is too small. Min area is {_2DWorld.MinBodySize}";
                return false;
            }

            if (area > _2DWorld.MaxBodySize)
            {
                errorMessage = $"Area is too large. Max area is {_2DWorld.MaxBodySize}";
                return false;
            }

            if (density < _2DWorld.MinDensity)
            {
                errorMessage = $"Density is too small. Min density area is {_2DWorld.MinDensity}";
                return false;
            }

            if (density > _2DWorld.MaxDensity)
            {
                errorMessage = $"Density is too large. Max density area is {_2DWorld.MaxDensity}";
                return false;
            }

            //force the restitution to be between 0 and 1
            restitution = _2DMath.Clamp(restitution, 0f, 1f);

            //calculate mass of circle (cylinder with a depth/height of 1)
            // mass = area * depth * density
            float mass = area * density;

            body = new _2DBody(position, density, mass, restitution, area, isStatic, 0f, width, height, ShapeType.Box);
            return true;
        }



    }
}

using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace _2DPhysics
{
    public readonly struct _2DVector
    {
        public readonly float X;
        public readonly float Y;
        public static readonly _2DVector Zero = new _2DVector(0f, 0f);

        public _2DVector(float x, float y)
        {
            this.X = x;
            this.Y = y;
        }

        public static _2DVector operator +(_2DVector a, _2DVector b)
        {
            return new _2DVector(a.X + b.X, a.Y + b.Y);
        }

        public static _2DVector operator -(_2DVector a, _2DVector b)
        {
            return new _2DVector(a.X - b.X, a.Y - b.Y);
        }

        public static _2DVector operator -(_2DVector v)
        {
            return new _2DVector(-v.X, -v.Y);
        }

        public static _2DVector operator *(_2DVector v, float s)
        {
            return new _2DVector(v.X * s, v.Y * s);
        }

        public static _2DVector operator *(float s, _2DVector v)
        {
            return new _2DVector(v.X * s, v.Y * s);
        }

        public static _2DVector operator /(_2DVector v, float s)
        {
            return new _2DVector(v.X / s, v.Y / s);
        }

        internal static _2DVector Transform(_2DVector v, _2DTransform transform)
        {
            //x1 = x0cos(θ) – y0sin(θ)
            //y1 = x0sin(θ) + y0cos(θ)

            //rotation
            float rx = transform.Cos * v.X - transform.Sin * v.Y;
            float ry = transform.Sin * v.X + transform.Cos * v.Y;

            //translation
            float tx = rx + transform.PositionX;
            float ty = ry + transform.PositionY;

            return new _2DVector(tx, ty);
        }
        
        public bool Equals(_2DVector other)
        {
            return this.X == other.X && this.Y == other.Y;
        }

        public override bool Equals(object obj)
        {
            if (obj is _2DVector other)
            {
                return this.Equals(other);
            }

            return false;
        }

        public override int GetHashCode()
        {
            return new { this.X, this.Y }.GetHashCode();
        }

        public override string ToString()
        {
            return $"X: {this.X}, Y: {this.Y}";
        }

    }
}

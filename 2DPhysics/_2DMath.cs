using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Metadata.Ecma335;
using System.Text;
using System.Threading.Tasks;

namespace _2DPhysics
{
    public static class _2DMath
    {
        public static readonly float HalfAMillimeter = 0.0005f;

        public static float Clamp(float value, float min, float max)
        {
            if (min == max)
            {
                return min;
            }

            if (min > max)
            {
                throw new ArgumentOutOfRangeException("min is greater than the max");
            }

            if (value < min)
            {
                return min;
            }

            if (value > max)
            {
                return max;
            }

            return value;
        }

        public static int Clamp(int value, int min, int max)
        {
            if (min == max)
            {
                return min;
            }

            if (min > max)
            {
                throw new ArgumentOutOfRangeException("min is greater than the max");
            }

            if (value < min)
            {
                return min;
            }

            if (value > max)
            {
                return max;
            }

            return value;
        }

        public static float Length(_2DVector v)
        {
            //float dx = v.X - 0f;
            //float dy = v.Y - 0f;

            return MathF.Sqrt(v.X * v.X + v.Y * v.Y);
        }

        public static float LengthSquared(_2DVector v)
        {
            return v.X * v.X + v.Y * v.Y;
        }

        //we're computing the distance between the ends of 2 vectors (points in space)
        //
        public static float Distance(_2DVector a, _2DVector b)
        {
            float dx = a.X - b.X; //"delta x"
            float dy = a.Y - b.Y; //"delta y"
            return MathF.Sqrt(dx * dx + dy * dy); //pythagorean theorem
        }

        public static float DistanceSquared(_2DVector a, _2DVector b)
        {
            float dx = a.X - b.X;
            float dy = a.Y - b.Y;

            return dx * dx + dy * dy;
        }

        //vectors have a direction and a magnitude
        //they point a certain way and we have to know how far we need to go in that direction
        //2d space,
        //makes the magnitude of the vector 1
        public static _2DVector Normalize(_2DVector v)
        {
            float len = _2DMath.Length(v);
            float x = v.X / len;
            float y = v.Y / len;
            return new _2DVector(x, y);
        }

        //dot product
        //"math is fun" dotproduct
        public static float Dot(_2DVector a, _2DVector b)
        {
            return a.X * b.X + a.Y * b.Y;
        }

        //3 dimensional cross product
        // cz = axby - aybx
        //geogebra.org/m/psMTGDgc ref
        public static float Cross(_2DVector a, _2DVector b)
        {
            return a.X * b.Y - a.Y * b.X;
        }

        //if less than half a millimeter apart
        public static bool NearlyEqual(float a, float b)
        {
            return Math.Abs(a - b) < _2DMath.HalfAMillimeter;
        }

        public static bool NearlyEqual(_2DVector a, _2DVector b)
        {
            return _2DMath.NearlyEqual(a.X, b.X) && _2DMath.NearlyEqual(a.Y, b.Y);
        }
    }
}

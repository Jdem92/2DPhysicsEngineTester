using System;
using System.Runtime.CompilerServices;
using Microsoft.Xna.Framework;
using _2DPhysics;
using System.Linq;

namespace _2DPhysicsGame
{
    public class _2DConverter
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 ToVector2(_2DVector v)
        {
            return new Vector2(v.X, v.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static _2DVector To_2DVector(Vector2 v)
        {
            return new _2DVector(v.X, v.Y);
        }

        public static void ToVector2Array(_2DVector[] src, ref Vector2[] dst)
        {
            if (dst == null || src.Length != dst.Length)
            {
                dst = new Vector2[src.Length];
            }

            for (int i = 0; i < src.Count(); i++)
            {
                _2DVector v = src[i];
                dst[i] = new Vector2(v.X, v.Y);
            }
        }


    }
}

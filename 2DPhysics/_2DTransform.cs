using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Metadata;
using System.Text;
using System.Threading.Tasks;

namespace _2DPhysics
{
    internal readonly struct _2DTransform
    {
        public readonly float PositionX;
        public readonly float PositionY;
        public readonly float Sin;
        public readonly float Cos;

        public readonly static _2DTransform Zero = new _2DTransform(0f, 0f, 0f);

        //
        public _2DTransform(_2DVector position, float angle)
        {
            this.PositionX = position.X;
            this.PositionY = position.Y;
            this.Sin = MathF.Sin(angle);
            this.Cos = MathF.Cos(angle);
        }

        //overloaded
        public _2DTransform(float x, float y, float angle)
        {
            this.PositionX = x;
            this.PositionY = y;
            this.Sin = MathF.Sin(angle);
            this.Cos = MathF.Cos(angle);
        }


    }
}

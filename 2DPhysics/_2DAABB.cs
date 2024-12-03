using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace _2DPhysics
{
    public readonly struct _2DAABB
    {
        public readonly _2DVector Min;
        public readonly _2DVector Max;

        public _2DAABB(_2DVector min, _2DVector max)
        {
            this.Min = min;
            this.Max = max;
        }

        public _2DAABB(float minX, float minY, float maxX, float maxY)
        {
            this.Min = new _2DVector(minX, minY);
            this.Max = new _2DVector(maxX, maxY);
        }
    }
}

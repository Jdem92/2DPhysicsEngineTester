using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace _2DPhysics
{
    //
    public readonly struct _2DManifold
    {
        public readonly _2DBody BodyA;
        public readonly _2DBody BodyB;
        public readonly _2DVector Normal;
        public readonly float Depth;
        public readonly _2DVector Contact1;
        public readonly _2DVector Contact2;
        public readonly int ContactCount;

        public _2DManifold(_2DBody bodyA, _2DBody bodyB, _2DVector normal, float depth, _2DVector contact1, _2DVector contact2, int contactCount)
        {
            this.BodyA = bodyA;
            this.BodyB = bodyB;
            this.Normal = normal;
            this.Depth = depth;
            this.Contact1 = contact1;
            this.Contact2 = contact2;
            this.ContactCount = contactCount;
        }
    }
}

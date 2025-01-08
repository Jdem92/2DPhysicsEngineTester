using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

namespace _2DPhysics
{
    public sealed class _2DWorld
    {
        public static readonly float MinBodySize = 0.01f * 0.01f;
        public static readonly float MaxBodySize = 64f * 64f;
        public static readonly float MinDensity = 0.5f; //g/cm^3
        public static readonly float MaxDensity = 21.4f; //

        //define min and max # of iterations allowed
        public static readonly int minIterations = 1;
        public static readonly int maxIterations = 128;

        private List<_2DBody> bodyList;
        private _2DVector gravity; //this is the acceleration due to gravity
        private List<_2DManifold> contactList;

        //debgging variables
        public List<_2DVector> ContactPointsList;

        public int BodyCount
        {
            get { return bodyList.Count; }
        }

        public _2DWorld()
        {
            this.gravity = new _2DVector(0f, -9.81f); //earth default
            this.bodyList = new List<_2DBody>();
            this.contactList = new List<_2DManifold>();

            this.ContactPointsList = new List<_2DVector>();
        }

        //we need functions to add / remove bodies & actually get the bodies out of the list if we need to 
        public void AddBody(_2DBody body)
        {
            this.bodyList.Add(body);
        }

        public bool RemoveBody(_2DBody body)
        {
            return this.bodyList.Remove(body);
        }

        //if index is within bounds of array return true
        public bool GetBody(int index, out _2DBody body)
        {
            body = null;

            if (index < 0 || index >= this.bodyList.Count())
            {
                return false;
            }

            body = this.bodyList[index];
            return true;
        }

        //
        public void Step(float time, int iterations)
        {
            //clamp # of iterations user has been through
            iterations = _2DMath.Clamp(iterations, _2DWorld.minIterations, _2DWorld.maxIterations);

            this.ContactPointsList.Clear();

            for (int it = 0; it < iterations; it++)
            {
                //movement step
                for (int i = 0; i < this.bodyList.Count(); i++)
                {
                    this.bodyList[i].Step(time, this.gravity, iterations);
                }

                this.contactList.Clear();

                //collision step
                for (int i = 0; i < this.bodyList.Count() - 1; i++)
                {
                    _2DBody bodyA = this.bodyList[i];
                    _2DAABB bodyA_aabb = bodyA.GetAABB();

                    for (int j = i + 1; j < this.bodyList.Count(); j++)
                    {
                        _2DBody bodyB = this.bodyList[j];
                        _2DAABB bodyB_aabb = bodyB.GetAABB();

                        //test axis aligned box for intersection
                        if (!Collisions.IntersectAABB(bodyA_aabb, bodyB_aabb))
                        {
                            continue;
                        }

                        if (bodyA.IsStatic && bodyB.IsStatic)
                        {
                            continue;
                        }

                        //found collision
                        if (Collisions.Collide(bodyA, bodyB, out _2DVector normal, out float depth))
                        {
                            //moving bodies apart
                            if (bodyA.IsStatic)
                            {
                                bodyB.Move(normal * depth);
                            }
                            else if (bodyB.IsStatic)
                            {
                                bodyA.Move(-normal * depth);
                            }
                            else
                            {
                                bodyA.Move(-normal * depth / 2f);
                                bodyB.Move(normal * depth / 2f);
                            }

                            Collisions.FindContactPoint(bodyA, bodyB, out _2DVector contactPoint1, out _2DVector contactPoint2, out int contactCount);

                            //saving contact information
                            _2DManifold contact = new _2DManifold(bodyA, bodyB, normal, depth, contactPoint1, contactPoint2, contactCount);
                            this.contactList.Add(contact);
                        }
                    }
                }

                for (int i = 0; i < this.contactList.Count(); i++)
                {
                    //resolve
                    _2DManifold contact = this.contactList[i];
                    this.ResolveCollision(contact);

                    if (contact.ContactCount > 0)
                    {
                        if (!this.ContactPointsList.Contains(contact.Contact1))
                        {
                            this.ContactPointsList.Add(contact.Contact1);
                        }

                        if (contact.ContactCount > 1)
                        {
                            if (!this.ContactPointsList.Contains(contact.Contact2))
                            {
                                this.ContactPointsList.Add(contact.Contact2);
                            }
                        }
                    }
                }
            }

            
        }

        //we will resolve collisions by applying "impulses"
        //"impulse" = 1 really quick adjustment to velocity to make objects move apart in a realistic manner
        // ex. Chris Hecker -> "Rigid body dynamics", "Collision response"
        public void ResolveCollision(in _2DManifold contact)
        {
            _2DBody bodyA = contact.BodyA;
            _2DBody bodyB = contact.BodyB;
            _2DVector normal = contact.Normal;
            float depth = contact.Depth;

            _2DVector relativeVelocity = bodyB.LinearVelocity - bodyA.LinearVelocity;

            if (_2DMath.Dot(relativeVelocity, normal) > 0f) //objects are moving apart already
            {
                return;
            }

            float e = MathF.Min(bodyA.Restitution, bodyB.Restitution);

            float j = -(1f + e) * _2DMath.Dot(relativeVelocity, normal);
            j /= bodyA.InvMass + bodyB.InvMass;

            _2DVector impulse = j * normal; //j is the magnitude of the impulse and the normal is the direction of the impulse

            bodyA.LinearVelocity -= impulse * bodyA.InvMass;
            bodyB.LinearVelocity += impulse * bodyB.InvMass;
        }


    }
}

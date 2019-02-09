using System;
using System.Collections.Generic;
using System.Text;

using MathNet.Numerics.LinearAlgebra.Double;

namespace ParticleConstrainedDynamics
{
    class Particle
    {
        public VectorN pos = new VectorN(2);
        public VectorN v = new VectorN(2);
        public VectorN forceAccum = new VectorN(2);
        public double mass;
        public double invMass;
        public bool immovable;
        public double radius;
        public bool freezed;

        public int id;

        private int framesToFreeze;
        public bool aboutToFreeze;

        public VectorN worldGravity;
        public VectorN curGravity;


        public Particle(int _id)
        {
            id = _id;
            immovable = false;
            freezed = false;
            aboutToFreeze = false;
        }


        public void SetMass(double m)
        {
            mass = m;
            invMass = 1 / m;
        }


        public void ApplyGravity()
        {
            forceAccum.AddScaled(curGravity, mass);
        }


        public void ApplyForce(VectorN f)
        {
            forceAccum.Add(f);
        }

        public void Integrate(double dt)
        {
            if (immovable || freezed) return;

            // Integrate velocity
            forceAccum.Scale(invMass);
            v.AddScaled(forceAccum, dt);

            // some damping
            v.Scale(Math.Pow(0.7, dt));

            // Integrate position
            pos.AddScaled(v, dt);

            Update();
        }



        public void Update()
        {
            if (immovable) return;

            double energy = v.Dot(v);
            if (energy < 0.01)
            {
                if (aboutToFreeze)
                {
                    //curGravity.Scale(0.8);

                    framesToFreeze++;
                    if (framesToFreeze > 20)
                    {
                        freezed = true;
                        aboutToFreeze = false;
                        v.Clear();
                    }
                }
                else
                {
                    framesToFreeze = 0;
                    aboutToFreeze = true;
                }
            }
            else
            {
                if (aboutToFreeze)
                {
                    aboutToFreeze = false;
                }
            }
        }

        public void Unfreeze()
        {
            freezed = false;
            aboutToFreeze = false;
            //curGravity.SetTo(worldGravity.v);
        }


    }
}

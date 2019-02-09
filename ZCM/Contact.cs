using System;
using System.Collections.Generic;
using System.Text;

namespace ParticleConstrainedDynamics
{
    class Contact : Joint
    {
        private VectorN normal;
        private double depth;
        private double restitution;
        public double relVel;


        public Contact(Particle p1, Particle p2, VectorN _normal, double _depth)
            : base(p1, p2)
        {
            normal = _normal;
            depth = _depth;
            restitution = 0.7;

            VectorN relVelVec = new VectorN(pair[0].v);
            relVelVec.Sub(pair[1].v);
            relVel = relVelVec.Dot(normal);
        }

        public override VectorN Jacobian()
        {
            J.v[0] = normal.v[0]; J.v[1] = normal.v[1];
            J.v[2] = -normal.v[0]; J.v[3] = -normal.v[1];

            if (relVel < 0.01) restitution = 0;

            J.v[4] = +depth * 60 - relVel * restitution;

            return J;
        }


        public void Unfreeze()
        {
            if (pair[0].immovable || pair[1].immovable) return;


            if (pair[0].freezed ^ pair[1].freezed)
            {
                if (relVel > 0.1)
                {
                    if (pair[0].freezed)
                    {
                        if (!pair[1].aboutToFreeze) pair[0].Unfreeze();
                    }
                    else if (pair[1].freezed)
                    {
                        if (!pair[0].aboutToFreeze) pair[1].Unfreeze();
                    }
                }

            }
            
        }

    }
}

using System;
using System.Collections.Generic;
using System.Text;

namespace ParticleConstrainedDynamics
{
    abstract class Joint
    {
        public Particle[] pair;
        protected VectorN J;


        public Joint(Particle p1, Particle p2)
        {
            pair = new Particle[2];
            pair[0] = p1;
            pair[1] = p2;

            J = new VectorN(5);
        }


        public abstract VectorN Jacobian();
        

    }
}

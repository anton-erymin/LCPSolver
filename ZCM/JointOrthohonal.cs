using System;
using System.Collections.Generic;
using System.Text;

namespace ParticleConstrainedDynamics
{
    class JointOrthogonal : Joint
    {

        public JointOrthogonal(Particle p1, Particle p2)
            : base(p1, p2)
        {
        }


        public override VectorN Jacobian()
        {
            J.v[0] = pair[1].v.v[0]; J.v[1] = pair[1].v.v[1];
            J.v[2] = 0; J.v[3] = 0;
            J.v[4] = 0;

            return J;
        }
    }
}

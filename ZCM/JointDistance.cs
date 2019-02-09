using System;
using System.Collections.Generic;
using System.Text;

namespace ParticleConstrainedDynamics
{
    class JointDistance : Joint
    {
        private double L;


        public JointDistance(Particle p1, Particle p2) : base(p1, p2)
        {
        }


        public void SetDistance(double distance)
        {
            L = distance;
        }


        public override VectorN Jacobian()
        {
            VectorN n = new VectorN(pair[1].pos);            
            n.Sub(pair[0].pos);
            double r = n.GetNorm();
            n.Scale(r);
            double c = r - L;

            J.v[0] = -n.v[0]; J.v[1] = -n.v[1];
            J.v[2] = n.v[0]; J.v[3] = n.v[1];
            J.v[4] = -0 * 200;

            return J;
        }

    }
}

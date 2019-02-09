using System;
using System.Collections.Generic;
using System.Text;

namespace ParticleConstrainedDynamics
{
    class VectorN
    {
        public uint n;
        public double[] v;


        public VectorN(uint _n)
        {
            n = _n;
            v = new double[n];
        }


        public VectorN(VectorN other)
        {
            n = other.n;
            v = new double[n];

            SetTo(other.v);
        }


        public void Clear()
        {
            for (int i = 0; i < n; i++) v[i] = 0.0;
        }


        public void SetTo(double[] val)
        {
            if (val.Length != n) return;

            for (int i = 0; i < n; i++) v[i] = val[i];
        }


        public void Scale(double s)
        {
            for (int i = 0; i < n; i++) v[i] *= s;
        }


        public VectorN ScaleR(double s)
        {
            VectorN res = new VectorN(this);
            res.Scale(s);
            return res;
        }


        public void Add(VectorN other)
        {
            if (other.n != n) return;

            for (int i = 0; i < n; i++) v[i] += other.v[i];
        }


        public VectorN AddR(VectorN other)
        {
            if (other.n != n) return null;

            VectorN res = new VectorN(this);
            res.Add(other);
            return res;
        }


        public void Sub(VectorN other)
        {
            if (other.n != n) return;

            for (int i = 0; i < n; i++) v[i] -= other.v[i];
        }


        public VectorN SubR(VectorN other)
        {
            if (other.n != n) return null;

            VectorN res = new VectorN(this);
            res.Sub(other);
            return res;
        }


        public void AddScaled(VectorN other, double s)
        {
            if (other.n != n) return;

            for (int i = 0; i < n; i++) v[i] += s * other.v[i];
        }


        public double GetNorm()
        {
            double res = 0;
            for (int i = 0; i < n; i++)
            {
                res += v[i] * v[i];
            }

            return Math.Sqrt(res);
        }


        public double GetNormSquared()
        {
            double res = 0;
            for (int i = 0; i < n; i++)
            {
                res += v[i] * v[i];
            }

            return res;
        }


        public double Normalize()
        {
            double norm = GetNorm();
            for (int i = 0; i < n; i++)
            {
                v[i] /= norm;
            }
            return norm;
        }


        public VectorN NormalizeR()
        {
            VectorN res = new VectorN(this);
            res.Normalize();
            return res;
        }


        public double Dot(VectorN other)
        {
            if (other.n != n) return 0;

            double res = 0;
            for (int i = 0; i < n; i++) res += v[i] * other.v[i];

            return res;
        }

    }
}

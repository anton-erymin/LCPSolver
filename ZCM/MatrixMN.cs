using System;
using System.Collections.Generic;
using System.Text;


namespace ParticleConstrainedDynamics
{
    class MatrixMN
    {
        public uint m;
        public uint n;
        public double[][] v;


        public MatrixMN(uint _m, uint _n)
        {
            m = _m;
            n = _n;
            v = new double[m][];
            for (int i = 0; i < m; i++) v[i] = new double[n];
        }


        public MatrixMN(MatrixMN other)
        {
            m = other.m;
            n = other.n;
            v = new double[m][];
            for (int i = 0; i < m; i++) v[i] = new double[n];

            SetTo(other.v);
        }


        public void Clear()
        {
            for (int i = 0; i < m; i++) 
                for (int j = 0; j < n; j++) v[i][j] = 0.0;
        }


        public void SetTo(double[][] val)
        {
            for (int i = 0; i < m; i++)
                for (int j = 0; j < n; j++) v[i][j] = val[i][j];
        }


        public void Identity()
        {
            if (m != n) return;

            Clear();
            for (int i = 0; i < m; i++)
                v[i][i] = 1.0;
        }


        public MatrixMN Multiply(MatrixMN other)
        {
            if (n != other.m) return null;

            MatrixMN res = new MatrixMN(m, other.n);

            for (int i = 0; i < m; i++)
                for (int j = 0; j < other.n; j++)
                {
                    res.v[i][j] = 0;
                    for (int k = 0; k < n; k++)
                        res.v[i][j] += v[i][k] * other.v[k][j];
                }

            return res;
        }


        public VectorN Multiply(VectorN vector)
        {
            if (n != vector.n) return null;

            VectorN res = new VectorN(m);

            double a;
            for (int i = 0; i < m; i++)
            {
                a = 0;
                for (int j = 0; j < n; j++)
                {
                    a += v[i][j] * vector.v[j];
                }
                res.v[i] = a;
            }

            return res;
        }


        public VectorN MultiplyTranspose(VectorN vector)
        {
            if (m != vector.n) return null;

            VectorN res = new VectorN(n);

            double a;
            for (int j = 0; j < n; j++)
            {
                a = 0;
                for (int i = 0; i < m; i++)
                {
                    a += v[i][j] * vector.v[i];
                }
                res.v[j] = a;
            }

            return res;
        }


        public MatrixMN Transpose()
        {
            MatrixMN res = new MatrixMN(n, m);

            for (int i = 0; i < m; i++)
            { 
                for (int j = 0; j < n; j++)
                {
                    res.v[j][i] = v[i][j];
                }
            }


            return res;
        }



        public static VectorN SolveGSSOR(MatrixMN A, VectorN b)
        {
            if (A.m != A.n || A.n != b.n) return null;

            VectorN x = new VectorN(b);

            double a;
            int numIter = 10;

            for (int iter = 0; iter < numIter; iter++)
            {
                for (int i = 0; i < A.n; i++)
                {
                    a = 0.0;
                    for (int j = 0; j < i; j++)       a += A.v[i][j] * x.v[j];
                    for (int j = i + 1; j < A.n; j++) a += A.v[i][j] * x.v[j];

                    x.v[i] += 1.2 * ((b.v[i] - a) / A.v[i][i] - x.v[i]);

                    if (x.v[i] < 0) x.v[i] = 0;
                }
            }

            return x;
        }

    }
}

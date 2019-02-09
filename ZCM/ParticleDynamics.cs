using System;
using System.Collections.Generic;
using System.Text;



namespace ParticleConstrainedDynamics
{
    class ParticleDynamics
    {
        public List<Particle> particles;
        public List<Joint> joints;
        public List<Contact> contacts;

        public VectorN gravity;

        public double CDTime;
        public double solverTime;


        public ParticleDynamics()
        {
            particles = new List<Particle>();
            joints = new List<Joint>();
            contacts = new List<Contact>();


            gravity = new VectorN(2);
            gravity.v[1] = -10.0;

        }


        public void AddParticle(Particle p)
        {
            if (p != null) particles.Add(p);
        }


        public void AddJoint(Joint joint)
        {
            if (joint != null) joints.Add(joint);
        }


        public Particle CreateParticle()
        {
            Particle p = new Particle(particles.Count);
            p.worldGravity = gravity;
            p.curGravity = new VectorN(gravity);
            AddParticle(p);
            return p;
        }


        public Joint CreateJointDistance(Particle p1, Particle p2, double distance)
        {
            JointDistance joint = new JointDistance(p1, p2);
            joint.SetDistance(distance);
            AddJoint(joint);
            return joint;
        }


        public Joint CreateJointOrthogonal(Particle p1, Particle p2)
        {
            JointOrthogonal joint = new JointOrthogonal(p1, p2);
            AddJoint(joint);
            return joint;
        }


        public void Simulate(double dt)
        {
            // Applying forces



            Collide();


            long t = DateTime.Now.Ticks;

            if (joints.Count == 0 && contacts.Count == 0)
            {
                foreach (Particle p in particles)
                {
                    p.Integrate(dt);
                }
                return;
            }


            uint n = (uint)particles.Count;
            int i, j;

            // Then build mass matrix M
            MatrixMN M = new MatrixMN(2 * n, 2 * n);
            for (i = 0, j = 0; i < n; i++, j += 2)
            {
                double mass = particles[i].invMass;
                if (particles[i].immovable) mass = 0;

                M.v[j    ][j    ] = mass;
                M.v[j + 1][j + 1] = mass;
            }

            // Build external force vector, velocity vector, position vector
            VectorN F = new VectorN(2 * n);
            VectorN V = new VectorN(2 * n);
            VectorN X = new VectorN(2 * n);

            for (i = 0, j = 0; i < n; i++, j += 2)
            {
                F.v[j    ] = particles[i].forceAccum.v[0];
                F.v[j + 1] = particles[i].forceAccum.v[1];

                if (particles[i].immovable)
                {
                    V.v[j    ] = 0;
                    V.v[j + 1] = 0;
                }
                else
                {
                    V.v[j    ] = particles[i].v.v[0];
                    V.v[j + 1] = particles[i].v.v[1];
                }

                X.v[j    ] = particles[i].pos.v[0];
                X.v[j + 1] = particles[i].pos.v[1];
            }



            // Build Jacobian J
            uint s = (uint)(joints.Count + contacts.Count);
            MatrixMN J = new MatrixMN(s, 2 * n);
            VectorN xi = new VectorN(s);
            for (i = 0; i < joints.Count; i++)
            {
                VectorN Jpartial = joints[i].Jacobian();

                Particle p = joints[i].pair[0];
                J.v[i][2 * p.id    ] = Jpartial.v[0];
                J.v[i][2 * p.id + 1] = Jpartial.v[1];

                p = joints[i].pair[1];
                J.v[i][2 * p.id    ] = Jpartial.v[2];
                J.v[i][2 * p.id + 1] = Jpartial.v[3];

                xi.v[i] = Jpartial.v[4];
            }

            for (j = 0, i = joints.Count; j < contacts.Count; j++, i++)
            {
                VectorN Jpartial = contacts[j].Jacobian();

                Particle p = contacts[j].pair[0];
                J.v[i][2 * p.id    ] = Jpartial.v[0];
                J.v[i][2 * p.id + 1] = Jpartial.v[1];

                p = contacts[j].pair[1];
                J.v[i][2 * p.id    ] = Jpartial.v[2];
                J.v[i][2 * p.id + 1] = Jpartial.v[3];

                xi.v[i] = Jpartial.v[4];
            }



            // Ax = b

            VectorN b = new VectorN(V);
            b.Scale(1 / dt);
            b.Sub(M.Multiply(F));
            b = J.Multiply(b);
            xi.Scale(1 / dt);
            b = xi.SubR(b);

            MatrixMN JT = J.Transpose();

            MatrixMN A = J.Multiply(M);
            A = A.Multiply(JT);


            // Solve Ax = b
            VectorN lambda = MatrixMN.SolveGSSOR(A, b);

            F.Add(JT.Multiply(lambda));
            //F = M.Multiply(F);
            //V.AddScaled(F, dt);
            //X.AddScaled(V, dt);


            for (i = 0, j = 0; i < n; i++, j += 2)
            {
                particles[i].forceAccum.v[0] = F.v[j];
                particles[i].forceAccum.v[1] = F.v[j + 1];

                particles[i].v.v[0] = V.v[j    ];
                particles[i].v.v[1] = V.v[j + 1];

                particles[i].pos.v[0] = X.v[j    ];
                particles[i].pos.v[1] = X.v[j + 1];

                particles[i].Integrate(dt);
                particles[i].forceAccum.Clear();
            }


            t = DateTime.Now.Ticks - t;
            solverTime = (double)t / (double)TimeSpan.TicksPerMillisecond;


        }


        public void Collide()
        {
            DateTime t = DateTime.Now;
            
            contacts.Clear();

            for (int i = 0; i < particles.Count - 1; i++)
                for (int j = i + 1; j < particles.Count; j++)
                {
                    if (particles[i].freezed && particles[j].freezed) continue;

                    double sum = particles[i].radius + particles[j].radius;
                    if (Math.Abs(particles[i].pos.v[0] - particles[j].pos.v[0]) > sum) continue;
                    if (Math.Abs(particles[i].pos.v[1] - particles[j].pos.v[1]) > sum) continue;


                    VectorN normal = new VectorN(particles[i].pos);
                    normal.Sub(particles[j].pos);
                    double dist = normal.Normalize();

                    
                    if (dist < sum)
                    {
                        Contact c = new Contact(particles[i], particles[j], normal, sum - dist);
                        c.Unfreeze();
                        contacts.Add(c);               
                    }

                }

            TimeSpan tt = DateTime.Now.Subtract(t);

            CDTime = (double)(tt.TotalMilliseconds);// / (double)TimeSpan.TicksPerMillisecond;

        }




        public void SimulateFast(double dt)
        {
            // Applying forces


            // Run CD
            Collide();

            DateTime t = DateTime.Now;


            if (joints.Count == 0 && contacts.Count == 0)
            {
                foreach (Particle p in particles)
                {
                    p.Integrate(dt);
                }
                return;
            }



            uint n = (uint)particles.Count;
            uint dofs = 0;
            int i, j;
            double idt = 1 / dt;

            //for (i = 0, j = 0; i < n; i++)
            //    if (!particles[i].freezed)
            //    {
            //        dofs++;
            //        particles[i].id = j;
            //        j++;
            //    }

            dofs = 2 * n;

            // Then build mass matrix M
            MatrixMN M = new MatrixMN(dofs, dofs);
            for (i = 0, j = 0; i < n; i++)
            {
                //if (particles[i].freezed) continue;

                double mass = particles[i].invMass;
                if (particles[i].immovable) mass = 0;

                M.v[j    ][j    ] = mass;
                M.v[j + 1][j + 1] = mass;

                j += 2;
            }

            // Build external force vector, velocity vector, position vector
            VectorN F = new VectorN(dofs);
            VectorN V = new VectorN(dofs);
            VectorN X = new VectorN(dofs);

            for (i = 0, j = 0; i < n; i++)
            {
                //if (particles[i].freezed) continue;

                F.v[j    ] = particles[i].forceAccum.v[0];
                F.v[j + 1] = particles[i].forceAccum.v[1];

                if (particles[i].immovable)
                {
                    V.v[j    ] = 0;
                    V.v[j + 1] = 0;
                }
                else
                {
                    V.v[j    ] = particles[i].v.v[0];
                    V.v[j + 1] = particles[i].v.v[1];
                }

                X.v[j    ] = particles[i].pos.v[0];
                X.v[j + 1] = particles[i].pos.v[1];

                j += 2;
            }




            // Build Jacobian J

            uint s = (uint)(joints.Count + contacts.Count);

            MatrixMN J = new MatrixMN(s, 4);
            int[][] Jmap = new int[s][];
            VectorN xi = new VectorN(s);

            for (i = 0; i < joints.Count; i++)
            {
                VectorN Jpartial = joints[i].Jacobian();

                J.v[i][0] = Jpartial.v[0];
                J.v[i][1] = Jpartial.v[1];
                J.v[i][2] = Jpartial.v[2];
                J.v[i][3] = Jpartial.v[3];

                Jmap[i] = new int[2];
                Jmap[i][0] = 2 * joints[i].pair[0].id;
                Jmap[i][1] = 2 * joints[i].pair[1].id;

                xi.v[i] = Jpartial.v[4];
            }

            for (j = 0, i = joints.Count; j < contacts.Count; j++, i++)
            {
                VectorN Jpartial = contacts[j].Jacobian();

                J.v[i][0] = Jpartial.v[0];
                J.v[i][1] = Jpartial.v[1];
                J.v[i][2] = Jpartial.v[2];
                J.v[i][3] = Jpartial.v[3];

                Jmap[i] = new int[2];
                Jmap[i][0] = 2 * contacts[j].pair[0].id;
                Jmap[i][1] = 2 * contacts[j].pair[1].id;

                xi.v[i] = Jpartial.v[4];
            }



            // 1/dt * V - MF
            VectorN q = new VectorN(dofs);
            for (i = 0; i < dofs; i++)
            {
                q.v[i] = idt * V.v[i] - M.v[i][i] * F.v[i];
            }

            // b = 1 / dt * xi - Jq     
            VectorN b = new VectorN(s);
            for (i = 0; i < s; i++)
            {
                int off1 = Jmap[i][0];
                int off2 = Jmap[i][1];

                b.v[i] = idt * xi.v[i] - (J.v[i][0] * q.v[off1] + J.v[i][1] * q.v[off1 + 1] + 
                                          J.v[i][2] * q.v[off2] + J.v[i][3] * q.v[off2 + 1]);
            }

            // A = M^-1 * J^T
            MatrixMN A = new MatrixMN(4, s);
            for (i = 0; i < s; i++)
            {
                int off1 = Jmap[i][0];
                int off2 = Jmap[i][1];

                A.v[0][i] = M.v[off1    ][off1]*J.v[i][0] + M.v[off1    ][off1 + 1]*J.v[i][1] + M.v[off1    ][off2]*J.v[i][2] + M.v[off1    ][off2 + 1]*J.v[i][3];
                A.v[1][i] = M.v[off1 + 1][off1]*J.v[i][0] + M.v[off1 + 1][off1 + 1]*J.v[i][1] + M.v[off1 + 1][off2]*J.v[i][2] + M.v[off1 + 1][off2 + 1]*J.v[i][3];
                A.v[2][i] = M.v[off2    ][off1]*J.v[i][0] + M.v[off2    ][off1 + 1]*J.v[i][1] + M.v[off2    ][off2]*J.v[i][2] + M.v[off2    ][off2 + 1]*J.v[i][3];
                A.v[3][i] = M.v[off2 + 1][off1]*J.v[i][0] + M.v[off2 + 1][off1 + 1]*J.v[i][1] + M.v[off2 + 1][off2]*J.v[i][2] + M.v[off2 + 1][off2 + 1]*J.v[i][3];
            }

            // Up to here all is right

            //MatrixMN Jfull = new MatrixMN(s, dofs);
            //MatrixMN Afull = new MatrixMN(dofs, s);
            //Jfull.Clear();
            //Afull.Clear();
            //for (i = 0; i < s; i++)
            //{
            //    int off1 = Jmap[i][0];
            //    int off2 = Jmap[i][1];

            //    Jfull.v[i][off1] = J.v[i][0];
            //    Jfull.v[i][off1 + 1] = J.v[i][1];
            //    Jfull.v[i][off2] = J.v[i][2];
            //    Jfull.v[i][off2 + 1] = J.v[i][3];

            //    Afull.v[off1][i] = A.v[0][i];
            //    Afull.v[off1 + 1][i] = A.v[1][i];
            //    Afull.v[off2][i] = A.v[2][i];
            //    Afull.v[off2 + 1][i] = A.v[3][i];
            //}

            //MatrixMN Z = Jfull.Multiply(Afull);
            //// Solve Ax = b
            //VectorN lambda = MatrixMN.SolveGSSOR(Z, b);
            //Jfull = Jfull.Transpose();
            //F.Add(Jfull.Multiply(lambda));

















            // Apparently there're a few bugs here...
            // Solve JAx = b
            // PGS-SOR starts here
            VectorN lambda = new VectorN(b);
            // a = A * lambda           
            // diag - diagonal elements of JA
            VectorN a = new VectorN(dofs);
            VectorN diag = new VectorN(s);
            a.Clear();
            for (i = 0; i < s; i++)
            {
                int off1 = Jmap[i][0];
                int off2 = Jmap[i][1];

                a.v[off1] += A.v[0][i] * lambda.v[i];
                a.v[off1 + 1] += A.v[1][i] * lambda.v[i];
                a.v[off2] += A.v[2][i] * lambda.v[i];
                a.v[off2 + 1] += A.v[3][i] * lambda.v[i];

                diag.v[i] = 1 / (J.v[i][0] * A.v[0][i] + J.v[i][1] * A.v[1][i] + J.v[i][2] * A.v[2][i] + J.v[i][3] * A.v[3][i]);
            }

            for (int iter = 0; iter < 10; iter++)
            {
                for (i = 0; i < s; i++)
                {
                    int off1 = Jmap[i][0];
                    int off2 = Jmap[i][1];

                    double l = b.v[i] - J.v[i][0] * a.v[off1] - J.v[i][1] * a.v[off1 + 1] - J.v[i][2] * a.v[off2] - J.v[i][3] * a.v[off2 + 1];
                    l *= diag.v[i];

                    double delta = lambda.v[i];
                    lambda.v[i] += 0.9 * (l - delta);

                    if (lambda.v[i] < 0) lambda.v[i] = 0;

                    delta = lambda.v[i] - delta;

                    a.v[off1] += delta * A.v[0][i];
                    a.v[off1 + 1] += delta * A.v[1][i];
                    a.v[off2] += delta * A.v[2][i];
                    a.v[off2 + 1] += delta * A.v[3][i];

                }
            }


            // F + J^T * lambda
            for (i = 0; i < s; i++)
            {
                int off1 = Jmap[i][0];
                int off2 = Jmap[i][1];

                F.v[off1] += J.v[i][0] * lambda.v[i];
                F.v[off1 + 1] += J.v[i][1] * lambda.v[i];
                F.v[off2] += J.v[i][2] * lambda.v[i];
                F.v[off2 + 1] += J.v[i][3] * lambda.v[i];
            }




            for (i = 0, j = 0; i < n; i++, j += 2)
            {
                particles[i].forceAccum.v[0] = F.v[j    ];
                particles[i].forceAccum.v[1] = F.v[j + 1];

                particles[i].v.v[0] = V.v[j    ];
                particles[i].v.v[1] = V.v[j + 1];

                particles[i].pos.v[0] = X.v[j    ];
                particles[i].pos.v[1] = X.v[j + 1];

                particles[i].Integrate(dt);

                particles[i].forceAccum.Clear();

            }



            TimeSpan tt = DateTime.Now.Subtract(t);
            
            solverTime = (double)(tt.TotalMilliseconds);// / (double)TimeSpan.TicksPerMillisecond;


        }







    }
}

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.Threading;


using ParticleConstrainedDynamics;


namespace ZCM
{
    public partial class Form1 : Form
    {
        private int WIDTH;
        private int HEIGHT;
        private int Ox;
        private int Oy;

        private double scale;

        private bool dragging = false;

        public Bitmap area;
        private Graphics g;


        private Pen blackp = new Pen(Color.Black, 1.0f);
        private Font blackf = new Font(FontFamily.GenericSansSerif, 12);


        private ParticleDynamics dynamics;
        private List<Particle> particles;



        private double leftBound;
        private double rightBound;
        private double topBound;
        private double bottomBound;
        private int areaWidth;
        private int areaHeight;

        private double mousex;
        private double mousey;

        private Particle dragged = null;


        public Form1()
        {      
            InitializeComponent();
            this.DoubleBuffered = true;
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            WIDTH = this.Width;
            HEIGHT = this.Height;

            Ox = WIDTH / 2;
            Oy = HEIGHT / 2;
            scale = 100;

            pbox.Width = WIDTH;
            pbox.Height = HEIGHT;
            

            area = new Bitmap(pbox.Width, pbox.Height);
            g = Graphics.FromImage(area);

            leftBound = (30 - Ox) / ((double)WIDTH / scale);
            topBound = -(30 - Oy) / ((double)WIDTH / scale);
            rightBound = (WIDTH - 40 - Ox) / ((double)WIDTH / scale);
            bottomBound = -(HEIGHT - 60 - Oy) / ((double)WIDTH / scale);
            areaWidth = WIDTH - 70;
            areaHeight = HEIGHT - 90;



            Random rand = new Random();

            dynamics = new ParticleDynamics();

            particles = new List<Particle>();


            // Build static world
            double wallRad = 1.5;
            double x = -45;
            double y = -41;
            for (int i = 0; i < 10; i++)
            {
                Particle p = dynamics.CreateParticle();
                p.pos.v[0] = x;
                p.pos.v[1] = y;
                y += 2 * wallRad;
                p.immovable = true;
                p.radius = wallRad;
                particles.Add(p);
            }

            x = -45 + 2 * wallRad + 0.3;
            y = -41;
            for (int i = 0; i < 29; i++)
            {
                Particle p = dynamics.CreateParticle();
                p.pos.v[0] = x;
                p.pos.v[1] = y;
                x += 2 * wallRad;
                p.immovable = true;
                p.radius = wallRad;
                particles.Add(p);
            }
            x = 45.3 - 2 * wallRad;
            y = -41 + 2 * wallRad;
            for (int i = 0; i < 10; i++)
            {
                Particle p = dynamics.CreateParticle();
                p.pos.v[0] = x;
                p.pos.v[1] = y;
                y += 2 * wallRad;
                p.immovable = true;
                p.radius = wallRad;
                particles.Add(p);
            }





            int NUM_PARTICLES = 30;
            double dist = 4.5;

            for (int i = 0; i < NUM_PARTICLES; i++)
            {
                Particle p = dynamics.CreateParticle();
                p.radius = dist;

                p.pos.v[0] = 80 * rand.NextDouble() - 40;
                p.pos.v[1] = 80 * rand.NextDouble() - 40;

                //p.pos.v[1] = -35 + (2 * dist + 0.1) * i;
                //p.pos.v[0] = 0;

                //p.v.v[0] = 50 * rand.NextDouble() - 25;
                //p.v.v[1] = 50 * rand.NextDouble() - 25;

                p.SetMass(10);

                if (i == 0) p.immovable = true;
                //if (i == 0) p.SetMass(1000);

                particles.Add(p);
            }


            //Pyramid
            //double px, py;
            //py = -35;
            //int row = 25;
            //double startx = -40;
            //for (int i = row; i >= 0; i--)
            //{
            //    px = startx + (2 * dist + 0.0 * dist) * (row - i);
            //    for (int j = 0; j <= i; j++)
            //    {
            //        Particle p = dynamics.CreateParticle();
            //        p.radius = dist;

            //        p.pos.v[1] = py;
            //        p.pos.v[0] = px;
            //        px += 2 * dist + 0.0 * dist;

            //        p.SetMass(10);
            //        if (i == row) p.immovable = true;
            //        particles.Add(p);
            //    }
            //    py += 2 * dist - 0.0 * dist;
            //}



            //Particle pa = dynamics.CreateParticle();
            //pa.radius = 0.8;

            ////p.pos.v[0] = 80 * rand.NextDouble() - 40;
            ////p.pos.v[1] = 80 * rand.NextDouble() - 40;

            //pa.pos.v[1] = -28;
            //pa.pos.v[0] = -40;

            ////pa.v.v[0] = 100;


            //pa.SetMass(400);

            //particles.Add(pa);




            //for (int i = 0; i < NUM_PARTICLES - 1; i++)
            //{
            //    for (int j = 0; j < NUM_PARTICLES; j++)
            //    {
            //        if (i == j) continue;
            //        Joint joint = dynamics.CreateJointDistance(particles[i], particles[i + 1], 2 * dist + 0.5);
            //    }
            //}

            
            

        }



        public void Simulate(double dt)
        {
            if (dragging)
            {
                VectorN dragForce = new VectorN(2);
                dragForce.v[0] = mousex - dragged.pos.v[0];
                dragForce.v[1] = mousey - dragged.pos.v[1];
                dragForce.Scale(50 * dragged.mass);
                dragged.ApplyForce(dragForce);
                dragged.Unfreeze();
            }


            for (int i = 0; i < particles.Count; i++)
            {
                if (particles[i] != dragged)
                    particles[i].ApplyGravity();
            }

            dynamics.SimulateFast(dt);


            CheckBounds();

        }


        private void CheckBounds()
        {
            double restitution = 0.6;
            double erp = 0.3;

            for (int i = 0; i < particles.Count; i++)
            {
                double[] pos = particles[i].pos.v;
                double r = particles[i].radius;

                if (pos[0] < leftBound + r || pos[0] > rightBound - r ||
                    pos[1] > topBound - r || pos[1] < bottomBound + r)
                {
                    double penetration = 0;
                    VectorN dir = new VectorN(2);

                    particles[i].v.Scale(restitution);

                    if (pos[0] < leftBound + r)
                    {
                        particles[i].v.v[0] *= -1;
                        penetration = leftBound - pos[0] + r;
                        dir.v[0] = 1;
                    }

                    


                    if (pos[0] > rightBound - r)
                    {
                        particles[i].v.v[0] *= -1;
                        penetration = pos[0] - rightBound + r;
                        dir.v[0] = -1;
                    }


                   

                    if (pos[1] > topBound - r)
                    {
                        particles[i].v.v[1] *= -1;
                        penetration = pos[1] - topBound + r;
                        dir.v[1] = -1;
                    }



                    if (pos[1] < bottomBound + r)
                    {
                        particles[i].v.v[1] *= -1;
                        penetration = bottomBound - pos[1] + r;
                        dir.v[1] = 1;
                    }


                    particles[i].v.Scale(restitution);


                    VectorN additional = particles[i].v.NormalizeR();
                    particles[i].v.Add(additional.ScaleR(penetration * erp));
                    
                    dir.Scale(penetration);
                    particles[i].pos.Add(dir);


                }

            }
        }
        


        public void Draw(int numSteps, int fps)
        {
            g.Clear(Color.White);


            double[] p;

            for (int i = 0; i < particles.Count; i++)
            {
                p = particles[i].pos.v;
                if (Double.IsNaN(p[0]) || Double.IsNaN(p[1])) {
                    continue;
                }
                int r = Sx(particles[i].radius) - Ox;
                
                if (particles[i].freezed)
                    g.DrawEllipse(Pens.LightGray, Sx(p[0]) - r, Sy(p[1]) - r, 2 * r, 2 * r);
                else g.DrawEllipse(Pens.Green, Sx(p[0]) - r, Sy(p[1]) - r, 2 * r, 2 * r);
            }

            g.DrawRectangle(blackp, Sx(leftBound), Sy(topBound), areaWidth, areaHeight);



            g.DrawString("CD time:      " + dynamics.CDTime + " ms", blackf, Brushes.Black, 40, 40);
            g.DrawString("Solver time: " + dynamics.solverTime + " ms", blackf, Brushes.Black, 40, 60);
            g.DrawString("N contacts: " + dynamics.contacts.Count, blackf, Brushes.Black, 40, 80);
            g.DrawString("N steps: " + numSteps, blackf, Brushes.Black, 40, 100);
            g.DrawString("fps: " + fps, blackf, Brushes.Black, 40, 120);


            pbox.Image = area;
        }


       






















        private void pbox_MouseDown(object sender, MouseEventArgs e)
        {
            mousex = (e.X - Ox) / ((double)WIDTH / scale);
            mousey = -(e.Y - Oy) / ((double)WIDTH / scale);

            foreach (Particle p in particles)
            {
                if (p.immovable) continue;

                double l = (mousex - p.pos.v[0]) * (mousex - p.pos.v[0]) + (mousey - p.pos.v[1]) * (mousey - p.pos.v[1]);
                if (l < p.radius * p.radius)
                {
                    dragging = true;
                    dragged = p;
                    break;
                }
            }

        }

        private void pbox_MouseMove(object sender, MouseEventArgs e)
        {
            if (dragging)
            {
                mousex = (e.X - Ox) / ((double)WIDTH / scale);
                mousey = -(e.Y - Oy) / ((double)WIDTH / scale);
            }
        }

        private void pbox_MouseUp(object sender, MouseEventArgs e)
        {
            dragging = false;
            dragged = null;
        }

        private void Form1_SizeChanged(object sender, EventArgs e)
        {
            WIDTH = this.Width;
            HEIGHT = this.Height;
            Ox = WIDTH / 2 - WIDTH / 8;
            Oy = HEIGHT / 2;
            scale = 60;
            pbox.Width = WIDTH;
            pbox.Height = HEIGHT;


            area = new Bitmap(pbox.Width, pbox.Height);
            g = Graphics.FromImage(area);

        }

        

        private int Sx(double _x)
        {
            int res = (int)(Ox + _x * (double)WIDTH / scale);
            return res;
        }

        private int Sy(double _y)
        {
            int res = (int)(Oy - _y * (double)WIDTH / scale);
            return res;
        }

    }

}

using System;
using System.Collections.Generic;
using System.Windows.Forms;
using System.Drawing;

namespace ZCM
{
    static class Program
    {

        private static bool mQuit;

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);

            Form1 f1 = new Form1();
            f1.FormClosed += QuitLoop;
            f1.Show();

            int lastTime, newTime;
            double dt = 1 / 60.0;
            double timeAcc = 0.0;
            double frameTime;
            lastTime = Environment.TickCount;
            int numSteps;
            int fps;

            do {
                Application.DoEvents();

                newTime = Environment.TickCount;
                frameTime = (newTime - lastTime) * 0.001;
                lastTime = newTime;

                fps = (int)(1.0 / frameTime);
                if (frameTime > 0.1) frameTime = 0.1;

                timeAcc += frameTime;

                numSteps = 0;
                while (timeAcc > dt)
                {
                    f1.Simulate(dt);
                    timeAcc -= dt;
                    numSteps++;
                }

                f1.Draw(numSteps, fps);


            } while (!mQuit);
        }


        private static void QuitLoop(object sender, FormClosedEventArgs e)
        {
            mQuit = true;
        }
        
    }
}

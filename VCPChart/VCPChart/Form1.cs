using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace VCPChart
{
    public partial class ChartForm : Form
    {
        private const int POINTS = 64;
        private const int MSG_SIZE = 256;
        private byte[] msg_buf = new byte[MSG_SIZE];

        private int count1;
        private int count2;
     //   private int[] arr1 = new int[POINTS];
     //   private int[] arr2 = new int[POINTS];

        public delegate void MessageRecievedDelegate();

        public ChartForm()
        {
            count1 = count2 = 0;
            InitializeComponent();
            chart1.Series[0].Points.Clear();
            chart1.Series[1].Points.Clear();
            chart1.ChartAreas[0].AxisY.Minimum = 0;
            chart1.ChartAreas[0].AxisY.Maximum = 4095;
            for (int i = 0; i < POINTS; i++)
            {
                chart1.Series[0].Points.AddXY(i, 100 + i);
                chart1.Series[1].Points.AddXY(i, 100 - i);
            }
        }

        private void ButtonRun_Click(object sender, EventArgs e)
        {
            if (!serialPort1.IsOpen)
            {
                try
                {
                    serialPort1.Open();
                }
                catch
                {
                    MessageBox.Show("Не удалось открыть порт", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                    return;
                }
                timer1.Start();
                serialPort1.Write("1"); //start adc
                toolStripStatusLabel1.Text = "Подключен";
                toolStripStatusLabel1.BackColor = Color.Green;
            }
        }

        private void stopAcquisition()
        {
            if (serialPort1.IsOpen)
            {
                try
                {
                    serialPort1.Write("0"); //stop adc
                }
                catch { };
                timer1.Stop();
                try
                {
                    serialPort1.Close();
                }
                catch { };
            }
            toolStripStatusLabel1.Text = "Нет связи";
            toolStripStatusLabel1.BackColor = Color.Red;
        }

        private void buttonStop_Click(object sender, EventArgs e)
        {
            stopAcquisition();
        }

        private void serialPort1_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            #region принимаем сообщение
            try
            {
                if (serialPort1.BytesToRead >= MSG_SIZE)
                {
                    serialPort1.Read(msg_buf, 0, MSG_SIZE);
                  //  messageRecieved();
                    BeginInvoke(new MessageRecievedDelegate(messageRecieved));
                } 
            }
            catch
            {
                stopAcquisition();
            }
            #endregion
        }

        public void messageRecieved()
        {
          /*  textBox1.Text = BitConverter.ToString(msg_buf, 0, 8) + System.Environment.NewLine;
            textBox1.AppendText(BitConverter.ToString(msg_buf, 8, 8) + System.Environment.NewLine);
            textBox1.AppendText(BitConverter.ToString(msg_buf, 16, 8) + System.Environment.NewLine);
            textBox1.AppendText(BitConverter.ToString(msg_buf, 24, 8) + System.Environment.NewLine);
            */
            #region отображаем данные
            chart1.Series[0].Points.SuspendUpdates();
            chart1.Series[1].Points.SuspendUpdates();
         //   textBox1.Clear();
            for (int i = 0; i < POINTS; i++)
            {
                UInt32 sample = BitConverter.ToUInt32(msg_buf, i * sizeof(UInt32));
                // UInt32 sample0 = sample /*& 0xFFF00000*/ >> 20;
                // UInt32 sample1 = (sample & 0x000FFF00) >> 8;

                UInt32 sample0 = 0;
                UInt32 sample1 = 0;
                if ((BitConverter.ToUInt32(msg_buf, 0) & 0x80000000) != 0)
                {
                    sample0 = sample & 0x00000FFF;
                    sample1 = (sample & 0x0FFF0000) >> 16;
                }


                // textBox1.AppendText(String.Format("ch A = {0}, ch B = {1}{2}", sample0, sample1, System.Environment.NewLine));
                chart1.Series[0].Points[i].SetValueY(sample0);
                chart1.Series[1].Points[i].SetValueY(sample1); 
            } 
            chart1.Series[0].Points.ResumeUpdates();
            chart1.Series[1].Points.ResumeUpdates();
            #endregion
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
#if false
            #region посылаем запрос данных
            count1++;
            chart1.Series[0].Points.SuspendUpdates();
            chart1.Series[1].Points.SuspendUpdates();
            /* for (int i = 0; i < POINTS; i++)
             {
                 arr1[i] = 100 + (int)(100 * Math.Sin(2 * Math.PI * (i + count1) / 125.0));
                 arr2[i] = 100 + (int)(100 * Math.Cos(2 * Math.PI * (i + count1) / 125.0));
                 chart1.Series[0].Points[i].SetValueY(arr1[i]);
                 chart1.Series[1].Points[i].SetValueY(arr2[i]);
             } */

            for (int i = 0; i < 64; i++)
            {
                arr1[i] = ;
                arr2[i] = ;
                chart1.Series[0].Points[i].SetValueY(arr1[i]);
                chart1.Series[1].Points[i].SetValueY(arr2[i]);
            }
            chart1.Series[0].Points.ResumeUpdates();
            chart1.Series[1].Points.ResumeUpdates();
            #endregion
#endif
        }

        private void timer2_Tick(object sender, EventArgs e)
        {
            #region FPS counter
            count2++;
            textBox1.Text = count1.ToString() + " " + count2.ToString() + " " + (count1 / count2).ToString();
            #endregion
        }

        private void ChartForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            stopAcquisition();
        }
    }
}

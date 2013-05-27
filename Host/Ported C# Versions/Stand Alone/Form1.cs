using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.IO.Ports;
using System.Windows.Forms;

namespace Bootloader
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Shown(object sender, EventArgs e)
        {
            cmbPorts.Items.Clear();
            cmbPorts.Items.AddRange(SerialPort.GetPortNames());
            cmbPorts.SelectedIndex = cmbPorts.Items.Count - 1;
        }

        private void button1_Click(object sender, EventArgs e)
        {
            OpenFileDialog aDialog = new OpenFileDialog();
            aDialog.Filter = "Compiled code (.hex)|*.hex";

            if (aDialog.ShowDialog() == DialogResult.OK)
            {
                if (aDialog.FileNames.Length>0)txtFile.Text = aDialog.FileNames[0];
            }
        }

        private void ResetBoard(string _ComPort)
        {
            SerialPort Port = new SerialPort(_ComPort, 921600, Parity.None, 8, StopBits.One);
            
            //Open the port
            Port.Open();

            //Send restart packet
             byte[] Packet = new byte[] { 0x02, 0x00, 0x00, 0x00, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x03 };
             Port.Write(Packet, 0, 37);

             Port.Close();
            //wait for board to restart
            System.Threading.Thread.Sleep(100);
        }

        private void button2_Click(object sender, EventArgs e)
        {
            //ResetBoard(cmbPorts.Text);

            xioBootloader myBootloader = new xioBootloader(cmbPorts.Text, txtFile.Text, new Point(this.Left+50, this.Top + 50));
            myBootloader.BootloadFinished += new xioBootloader.OnBootloadFinished(myBootloader_BootloadFinished);

            try
            {
                myBootloader.DoBootload();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message + "\n" + ex.StackTrace, "Error occured");
            }
        }

        void myBootloader_BootloadFinished(object sender)
        {
            MessageBox.Show("Bootloading finished successfully.");
        }

    }
}

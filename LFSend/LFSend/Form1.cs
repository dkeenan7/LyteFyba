using System;
using System.IO;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace LFSend
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
            UpdateFeedback();
        }

        private void button3_Click(object sender, EventArgs e)
        {

        }

        private void OpenButton_Click(object sender, EventArgs e)
        {
            OpenFileDialog of = new OpenFileDialog();
            BinaryReader brd;
            Stream strm;

            of.Filter = "binary files (*.bin)|*.bin|hex files (*.hex)|*.hex|All files|*.*";
            of.FilterIndex = 1;
            of.RestoreDirectory = true;
            if (of.ShowDialog() == DialogResult.OK)
            {
                try
                {
                    strm = of.OpenFile();
                    brd = new BinaryReader(strm);
                    brd.Read(m_FileBuf, 0, (int)strm.Length);
                    m_FileName = of.SafeFileName;
                    UpdateFeedback();
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Error: Could not read file from disk. Original error: " + ex.Message);
                }
            }
        }

        byte[] m_FileBuf = new byte[8192];
        string m_FileName = "unknown file";
        string m_SerialName = "unknown port";
    }
}

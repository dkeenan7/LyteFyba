namespace LFSend
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.button1 = new System.Windows.Forms.Button();
            this.OpenButton = new System.Windows.Forms.Button();
            this.button3 = new System.Windows.Forms.Button();
            this.FeedbackText = new System.Windows.Forms.TextBox();
            this.SuspendLayout();
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(139, 201);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 0;
            this.button1.Text = "&Send";
            this.button1.UseVisualStyleBackColor = true;
            // 
            // OpenButton
            // 
            this.OpenButton.Location = new System.Drawing.Point(139, 113);
            this.OpenButton.Name = "OpenButton";
            this.OpenButton.Size = new System.Drawing.Size(75, 23);
            this.OpenButton.TabIndex = 1;
            this.OpenButton.Text = "&Open";
            this.OpenButton.UseVisualStyleBackColor = true;
            this.OpenButton.Click += new System.EventHandler(this.OpenButton_Click);
            // 
            // button3
            // 
            this.button3.Location = new System.Drawing.Point(139, 155);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(75, 23);
            this.button3.TabIndex = 2;
            this.button3.Text = "S&elect serial";
            this.button3.UseVisualStyleBackColor = true;
            this.button3.Click += new System.EventHandler(this.button3_Click);
            // 
            // FeedbackText
            // 
            this.FeedbackText.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.FeedbackText.Location = new System.Drawing.Point(12, 41);
            this.FeedbackText.Name = "FeedbackText";
            this.FeedbackText.Size = new System.Drawing.Size(338, 26);
            this.FeedbackText.TabIndex = 3;
            this.FeedbackText.Text = "Sending unknown to port unknown";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(362, 261);
            this.Controls.Add(this.FeedbackText);
            this.Controls.Add(this.button3);
            this.Controls.Add(this.OpenButton);
            this.Controls.Add(this.button1);
            this.Name = "Form1";
            this.Text = "LFSend";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        private void UpdateFeedback()
        {
            FeedbackText.Text = "Sending " + m_FileName + " to " + m_SerialName;
        }
        #endregion

        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Button OpenButton;
        private System.Windows.Forms.Button button3;
        private System.Windows.Forms.TextBox FeedbackText;
    }
}


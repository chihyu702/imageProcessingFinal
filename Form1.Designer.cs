using System.Drawing;
using System.Threading;
using System.Windows.Forms;

namespace INFOIBV
{
    partial class INFOIBV
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
            this.LoadImageButton = new System.Windows.Forms.Button();
            this.openImageDialog = new System.Windows.Forms.OpenFileDialog();
            this.imageFileName = new System.Windows.Forms.TextBox();
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.applyButton = new System.Windows.Forms.Button();
            this.saveImageDialog = new System.Windows.Forms.SaveFileDialog();
            this.saveButton = new System.Windows.Forms.Button();
            this.pictureBox2 = new System.Windows.Forms.PictureBox();
            this.progressBar = new System.Windows.Forms.ProgressBar();
            this.comboBox = new System.Windows.Forms.ComboBox();
            this.pictureBox3 = new System.Windows.Forms.PictureBox(); // Declare new PictureBox
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox3)).BeginInit(); // Initialize new PictureBox
            this.distinctValueLabel = new System.Windows.Forms.Label(); // Declare new label of the text
            this.boundaryLabel = new System.Windows.Forms.TextBox(); // Declare new textbox for boundary tracing
            this.peakValues = new System.Windows.Forms.Label(); // declare new label for the peak values
            this.lineSegmentsLabel = new System.Windows.Forms.Label(); // declare new label for the line segments







     



            this.SuspendLayout();
            // 
            // LoadImageButton
            // 
            this.LoadImageButton.Location = new System.Drawing.Point(12, 11);
            this.LoadImageButton.Name = "LoadImageButton";
            this.LoadImageButton.Size = new System.Drawing.Size(98, 23);
            this.LoadImageButton.TabIndex = 0;
            this.LoadImageButton.Text = "Load image...";
            this.LoadImageButton.UseVisualStyleBackColor = true;
            this.LoadImageButton.Click += new System.EventHandler(this.loadImageButton_Click);
            // 
            // openImageDialog
            // 
            this.openImageDialog.Filter = "Bitmap files (*.bmp;*.gif;*.jpg;*.png;*.tiff;*.jpeg)|*.bmp;*.gif;*.jpg;*.png;*.ti" +
    "ff;*.jpeg";
            this.openImageDialog.InitialDirectory = "..\\..\\images";
            // 
            // imageFileName
            // 
            this.imageFileName.Location = new System.Drawing.Point(116, 12);
            this.imageFileName.Name = "imageFileName";
            this.imageFileName.ReadOnly = true;
            this.imageFileName.Size = new System.Drawing.Size(329, 20);
            this.imageFileName.TabIndex = 1;
            // 
            // pictureBox1
            // 
            this.pictureBox1.Location = new System.Drawing.Point(13, 45);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(512, 512);
            this.pictureBox1.SizeMode = System.Windows.Forms.PictureBoxSizeMode.CenterImage;
            this.pictureBox1.TabIndex = 2;
            this.pictureBox1.TabStop = false;
            // 
            // applyButton
            // 
            this.applyButton.Location = new System.Drawing.Point(658, 11);
            this.applyButton.Name = "applyButton";
            this.applyButton.Size = new System.Drawing.Size(103, 23);
            this.applyButton.TabIndex = 3;
            this.applyButton.Text = "Apply";
            this.applyButton.UseVisualStyleBackColor = true;
            this.applyButton.Click += new System.EventHandler(this.applyButton_Click);
            // 
            // saveImageDialog
            // 
            this.saveImageDialog.Filter = "Bitmap file (*.bmp)|*.bmp";
            this.saveImageDialog.InitialDirectory = "..\\..\\images";
            // 
            // saveButton
            // 
            this.saveButton.Location = new System.Drawing.Point(948, 11);
            this.saveButton.Name = "saveButton";
            this.saveButton.Size = new System.Drawing.Size(95, 23);
            this.saveButton.TabIndex = 4;
            this.saveButton.Text = "Save as BMP...";
            this.saveButton.UseVisualStyleBackColor = true;
            this.saveButton.Click += new System.EventHandler(this.saveButton_Click);
            // 
            // pictureBox2
            // 
            this.pictureBox2.Location = new System.Drawing.Point(413, 45);
            this.pictureBox2.Name = "pictureBox2";
            this.pictureBox2.Size = new System.Drawing.Size(512, 512);
            this.pictureBox2.SizeMode = System.Windows.Forms.PictureBoxSizeMode.CenterImage;
            this.pictureBox2.TabIndex = 5;
            this.pictureBox2.TabStop = false;
            // 
            // progressBar
            // 
            this.progressBar.Location = new System.Drawing.Point(767, 13);
            this.progressBar.Name = "progressBar";
            this.progressBar.Size = new System.Drawing.Size(175, 20);
            this.progressBar.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            this.progressBar.TabIndex = 6;
            this.progressBar.Visible = false;
            // 
            // comboBox
            // 
            this.comboBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.comboBox.FormattingEnabled = true;
            this.comboBox.Location = new System.Drawing.Point(451, 12);
            this.comboBox.Name = "comboBox";
            this.comboBox.Size = new System.Drawing.Size(201, 21);
            this.comboBox.TabIndex = 7;
            // 
            // pictureBox3
            // 
            this.pictureBox3.Location = new System.Drawing.Point(813, 45); // Set the location of the third PictureBox
            this.pictureBox3.Name = "pictureBox3";
            this.pictureBox3.Size = new System.Drawing.Size(512, 512); // Set the size of the third PictureBox
            this.pictureBox3.SizeMode = System.Windows.Forms.PictureBoxSizeMode.CenterImage;
            this.pictureBox3.TabIndex = 8; // Set the tab index for the third PictureBox
            this.pictureBox3.TabStop = false;
            // 
            // distinctValueLabel
            // 
            this.distinctValueLabel.AutoSize = true;
            this.distinctValueLabel.Location = new System.Drawing.Point(600, 580);
            this.distinctValueLabel.Name = "distinctValueLabel";
            this.distinctValueLabel.Size = new System.Drawing.Size(512, 512);
            this.distinctValueLabel.TabIndex = 20;
            this.Controls.Add(this.distinctValueLabel);
            // 
            // peakValueLabel
            // 
            // Make the label as large as possible
            this.peakValues.AutoSize = true;
            this.peakValues.Location = new System.Drawing.Point(600, 180);
            this.peakValues.Name = "peakValueLabel";
            this.peakValues.Size = new System.Drawing.Size(512, 1080);
            this.peakValues.TabIndex = 20;
            this.Controls.Add(this.peakValues);


            this.Controls.Add(this.peakValues);

            // 
            // boundaryTextBox
            // 
            this.boundaryLabel.Location = new System.Drawing.Point(1000, 45);
            this.boundaryLabel.Multiline = true;
            this.boundaryLabel.Size = new System.Drawing.Size(300, 512);
            this.boundaryLabel.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.boundaryLabel.ReadOnly = true;
            this.boundaryLabel.Visible = false;
            this.Controls.Add(this.boundaryLabel);

            // 
            // lineSegmentsLabel
            // 
            // Make the label as large as possible
            this.lineSegmentsLabel.AutoSize = true;
            this.lineSegmentsLabel.Location = new System.Drawing.Point(600, 180);
            this.lineSegmentsLabel.Name = "lineSegmentsLabel";
            this.lineSegmentsLabel.Size = new System.Drawing.Size(512, 1080);
            this.lineSegmentsLabel.TabIndex = 20;
            this.Controls.Add(this.lineSegmentsLabel);


            // 
            // peakThresholdLabel
            // 
            this.peakThresholdLabel = new System.Windows.Forms.Label();
            this.peakThresholdLabel.AutoSize = true;
            this.peakThresholdLabel.Location = new System.Drawing.Point(50, 610);
            this.peakThresholdLabel.Name = "peakThresholdLabel";
            this.peakThresholdLabel.Size = new System.Drawing.Size(120, 13);
            this.peakThresholdLabel.Text = "Peak Threshold:";
            this.Controls.Add(this.peakThresholdLabel);

            // 
            // peakThresholdTextBox
            // 
            this.peakThresholdTextBox = new System.Windows.Forms.TextBox();
            this.peakThresholdTextBox.Location = new System.Drawing.Point(200, 610);
            this.peakThresholdTextBox.Size = new System.Drawing.Size(100, 20);
            this.peakThresholdTextBox.Name = "peakThresholdTextBox";
            this.Controls.Add(this.peakThresholdTextBox);



            // 
            // intensityThresholdLabel
            // 
            this.intensityThresholdLabel = new System.Windows.Forms.Label();
            this.intensityThresholdLabel.AutoSize = true;
            this.intensityThresholdLabel.Location = new System.Drawing.Point(50, 580);
            this.intensityThresholdLabel.Name = "intensityThresholdLabel";
            this.intensityThresholdLabel.Size = new System.Drawing.Size(120, 13);
            this.intensityThresholdLabel.Text = "Minimum Intensity Threshold:";
            this.Controls.Add(this.intensityThresholdLabel);

            // 
            // intensityThresholdTextBox
            // 
            this.intensityThresholdTextBox = new System.Windows.Forms.TextBox();
            this.intensityThresholdTextBox.Location = new System.Drawing.Point(200, 580);
            this.intensityThresholdTextBox.Size = new System.Drawing.Size(100, 20);
            this.intensityThresholdTextBox.Name = "intensityThresholdTextBox";
            this.Controls.Add(this.intensityThresholdTextBox);

            // 
            // minLengthLabel
            // 
            this.minLengthLabel = new System.Windows.Forms.Label();
            this.minLengthLabel.AutoSize = true;
            this.minLengthLabel.Location = new System.Drawing.Point(400, 580);
            this.minLengthLabel.Name = "minLengthLabel";
            this.minLengthLabel.Size = new System.Drawing.Size(120, 13);
            this.minLengthLabel.Text = "Minimum Length:";
            this.Controls.Add(this.minLengthLabel);

            // 
            // minLengthTextBox
            // 
            this.minLengthTextBox = new System.Windows.Forms.TextBox();
            this.minLengthTextBox.Location = new System.Drawing.Point(490, 580);
            this.minLengthTextBox.Size = new System.Drawing.Size(100, 20);
            this.minLengthTextBox.Name = "minLengthTextBox";
            this.Controls.Add(this.minLengthTextBox);

            // 
            // maxGapLabel
            // 
            this.maxGapLabel = new System.Windows.Forms.Label();
            this.maxGapLabel.AutoSize = true;
            this.maxGapLabel.Location = new System.Drawing.Point(600, 580);
            this.maxGapLabel.Name = "maxGapLabel";
            this.maxGapLabel.Size = new System.Drawing.Size(120, 13);
            this.maxGapLabel.Text = "Maximum Gap:";
            this.Controls.Add(this.maxGapLabel);

            // 
            // maxGapTextBox
            // 
            this.maxGapTextBox = new System.Windows.Forms.TextBox();
            this.maxGapTextBox.Location = new System.Drawing.Point(700, 580);
            this.maxGapTextBox.Size = new System.Drawing.Size(100, 20);
            this.maxGapTextBox.Name = "maxGapTextBox";
            this.Controls.Add(this.maxGapTextBox);

            // 
            // angleLimitsLabel
            // 
            this.angleLimitsLabel = new System.Windows.Forms.Label();
            this.angleLimitsLabel.AutoSize = true;
            this.angleLimitsLabel.Location = new System.Drawing.Point(950, 580);
            this.angleLimitsLabel.Name = "angleLimitsLabel";
            this.angleLimitsLabel.Size = new System.Drawing.Size(120, 13);
            this.angleLimitsLabel.Text = "Angle Limits (min, max):";
            this.Controls.Add(this.angleLimitsLabel);

            // 
            // angleLimitsTextBox
            // 
            this.angleLimitsTextBox = new System.Windows.Forms.TextBox();
            this.angleLimitsTextBox.Location = new System.Drawing.Point(1100, 580);
            this.angleLimitsTextBox.Size = new System.Drawing.Size(100, 20);
            this.angleLimitsTextBox.Name = "angleLimitsTextBox";
            this.Controls.Add(this.angleLimitsTextBox);



            // 
            // INFOIBV
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1575, 576); // adjust the form size to fit the new picture box
            this.Controls.Add(this.pictureBox3); // Add the third PictureBox to the form
            this.Controls.Add(this.comboBox);
            this.Controls.Add(this.progressBar);
            this.Controls.Add(this.pictureBox2);
            this.Controls.Add(this.saveButton);
            this.Controls.Add(this.applyButton);
            this.Controls.Add(this.pictureBox1);
            this.Controls.Add(this.imageFileName);
            this.Controls.Add(this.LoadImageButton);
            this.Location = new System.Drawing.Point(10, 10);
            this.Name = "INFOIBV";
            this.ShowIcon = false;
            this.Text = "INFOIBV";
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox3)).EndInit(); // End initialization for the third PictureBox
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button LoadImageButton;
        private System.Windows.Forms.OpenFileDialog openImageDialog;
        private System.Windows.Forms.TextBox imageFileName;
        private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.Button applyButton;
        private System.Windows.Forms.SaveFileDialog saveImageDialog;
        private System.Windows.Forms.Button saveButton;
        private System.Windows.Forms.PictureBox pictureBox2;
        private System.Windows.Forms.ProgressBar progressBar;
        private System.Windows.Forms.ComboBox comboBox;
        private System.Windows.Forms.PictureBox pictureBox3; // declare the new PictureBox
        private System.Windows.Forms.Label distinctValueLabel; // declare the text of the distinct pixel value
        private System.Windows.Forms.TextBox boundaryLabel; // declare the textBox of the boundary tracing 
        private System.Windows.Forms.Label peakValues; // declare text of peak values
        private System.Windows.Forms.Label lineSegmentsLabel; // declare text of line segments


        private System.Windows.Forms.TextBox intensityThresholdTextBox;
        private System.Windows.Forms.Label intensityThresholdLabel;
        private System.Windows.Forms.TextBox minLengthTextBox;
        private System.Windows.Forms.Label minLengthLabel;
        private System.Windows.Forms.TextBox maxGapTextBox;
        private System.Windows.Forms.Label maxGapLabel;
        private System.Windows.Forms.TextBox angleLimitsTextBox;
        private System.Windows.Forms.Label angleLimitsLabel;

        private System.Windows.Forms.TextBox peakThresholdTextBox;
        private System.Windows.Forms.Label peakThresholdLabel;

    }
}
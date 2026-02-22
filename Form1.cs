using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO;



using System.Text.RegularExpressions;
using static System.Net.WebRequestMethods;

namespace INFOIBV
{
    public partial class INFOIBV : Form
    {
        private Bitmap InputImage;
        private Bitmap SecondImage;
        private Bitmap OutputImage;

        /*
         * this enum defines the processing functions that will be shown in the dropdown (a.k.a. combobox)
         * you can expand it by adding new entries to applyProcessingFunction()
         */
        private enum ProcessingFunctions
        {
            Invert,
            AdjustContrast,
            ConvolutionFilter,
            MedianFilter,
            DetectEdges,
            Threshold,
            EdgeSharpening,
            HistogramEqualization,
            dilateImage,
            erodeImage,
            openImage,
            closeImage,
            andImages,
            orImages,
            valueCounting,
            traceBoundary,
            largestShape,
            complementImage,
            CountNonBackgroundValues,
            HoughTransform,
            peakFinding,
            houghTransformAngleLimits,
            houglineDetection,
            visualizeHoughLineSegments,
            houghTransformForCircles,
            preProcessing,
            detectStopSign
        }

        /*
         * these are the parameters for your processing functions, you should add more as you see fit
         * it is useful to set them based on controls such as sliders, which you can add to the form
         */
        private byte filterSize = 7;
        private float filterSigma = 1f;
        private byte threshold = 127;
        private byte structureElementSize = 5;
        private int closingSize = 7; // size for morphological closing

        private string[] angleLimits;
        private double lowerAngleLimit;
        private double upperAngleLimit;

        private int peakThreshold;
        private int minIntensityThreshold;
        private int minLength;
        private int maxGap;


        // define these kernels yourself
        // sobel edge detectors for gx and gy
        private sbyte[,] horizontalKernel = new sbyte[,]
        {
                        {1, 2, 1},
                        {0, 0, 0},
                        {-1, -2, -1}
        };
        private sbyte[,] verticalKernel = new sbyte[,]
        {
                        {-1, 0, 1},
                        {-2, 0, 2},
                        {-1, 0, 1}
        };



        public INFOIBV()
        {
            InitializeComponent();
            populateCombobox();
        }

        // Method to initialize parameters (call this in the applyButton_Click event)
        private bool InitializeParameters()
        {
            // Use TryParse so empty boxes don't block simple operations (Invert, Blur, etc.)
            int.TryParse(peakThresholdTextBox.Text, out peakThreshold);
            int.TryParse(intensityThresholdTextBox.Text, out minIntensityThreshold);
            int.TryParse(minLengthTextBox.Text, out minLength);
            int.TryParse(maxGapTextBox.Text, out maxGap);
            return true;
        }

        private bool TryParseAngleLimits()
        {
            angleLimits = angleLimitsTextBox.Text.Split(',');
            if (angleLimits.Length != 2)
            {
                MessageBox.Show("Invalid input: Please provide two angle limits separated by a comma.", "Input Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return false;
            }
            if (!double.TryParse(angleLimits[0], out lowerAngleLimit) || !double.TryParse(angleLimits[1], out upperAngleLimit))
            {
                MessageBox.Show("Invalid input: Angle limits must be numeric values.", "Input Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return false;
            }
            return true;
        }



        /*
         * populateCombobox: populates the combobox with items as defined by the ProcessingFunctions enum
         */
        private void populateCombobox()
        {
            foreach (string itemName in Enum.GetNames(typeof(ProcessingFunctions)))
            {
                string ItemNameSpaces = Regex.Replace(Regex.Replace(itemName, @"(\P{Ll})(\P{Ll}\p{Ll})", "$1 $2"), @"(\p{Ll})(\P{Ll})", "$1 $2");
                comboBox.Items.Add(ItemNameSpaces);
            }
            comboBox.SelectedIndex = 0;
        }

        /*
        * loadButton_Click: process when user clicks "Load" button
        */
        private void loadImageButton_Click(object sender, EventArgs e)
        {
            if (openImageDialog.ShowDialog() == DialogResult.OK) // open file dialog
            {
                string file = openImageDialog.FileName;          // get the file name
                imageFileName.Text = file;                       // show file name

                Bitmap newImage = new Bitmap(file);              // create new Bitmap from file

                // Validate image dimensions (optional)
                if (newImage.Size.Height <= 0 || newImage.Size.Width <= 0 ||
                    newImage.Size.Height > 512 || newImage.Size.Width > 512)
                {
                    MessageBox.Show("Error in image dimensions (have to be > 0 and <= 512)");
                    newImage.Dispose(); // Dispose the invalid image
                    return;
                }

                // Check if InputImage is already set
                if (InputImage == null)
                {
                    InputImage = newImage;                      // Set InputImage
                    pictureBox1.Image = (Image)InputImage;      // Display in pictureBox1
                }

                else if (SecondImage == null)
                {
                    SecondImage = newImage;                     // Set SecondImage
                    pictureBox2.Image = (Image)SecondImage;     // Display in pictureBox2
                }

                else
                {
                    MessageBox.Show("Both images are already loaded. Please clear one before loading a new image.");
                    newImage.Dispose(); // Dispose the image since both slots are full
                }
            }
        }



        private void applyButton_Click(object sender, EventArgs e)
        {
            if (InputImage == null)
            {
                MessageBox.Show("No input image loaded.");
                return; // exit if no input image
            }

            // Initialize parameters from the text boxes
            if (!InitializeParameters()) return;

            if (OutputImage != null) OutputImage.Dispose(); // reset output image
            OutputImage = new Bitmap(InputImage.Size.Width, InputImage.Size.Height); // create new output image

            // Create array to speed-up operations (Bitmap functions are very slow)
            Color[,] Image = new Color[InputImage.Size.Width, InputImage.Size.Height];

            // Copy input Bitmap to array
            for (int x = 0; x < InputImage.Size.Width; x++)
            {
                for (int y = 0; y < InputImage.Size.Height; y++)
                {
                    Image[x, y] = InputImage.GetPixel(x, y); // set pixel color in array at (x, y)
                }
            }

            // Convert first image to grayscale (0-255)
            byte[,] workingImage = convertToGrayscale(Image);

            byte[,] secondImage = null; // Initialize secondImage as null

            // Only process SecondImage if it exists
            if (SecondImage != null)
            {
                // Create array for SecondImage
                Color[,] Image2 = new Color[SecondImage.Size.Width, SecondImage.Size.Height];

                // Copy SecondImage Bitmap to array
                for (int x = 0; x < SecondImage.Size.Width; x++)
                {
                    for (int y = 0; y < SecondImage.Size.Height; y++)
                    {
                        Image2[x, y] = SecondImage.GetPixel(x, y); // set pixel color in array at (x, y)
                    }
                }

                secondImage = convertToGrayscale(Image2); // Convert SecondImage to grayscale
            }

            // Apply processing with or without the second image
            workingImage = applyProcessingFunction(workingImage, secondImage); // Pass null if secondImage is not loaded

            if (workingImage == null) return; // processing was cancelled or failed

            // Copy array to output Bitmap
            for (int x = 0; x < workingImage.GetLength(0); x++)
            {
                for (int y = 0; y < workingImage.GetLength(1); y++)
                {
                    // Ensure we are using valid values for setting pixels
                    if (x >= 0 && x < OutputImage.Width && y >= 0 && y < OutputImage.Height)
                    {
                        // Map the grayscale value back to color for the output image
                        Color newColor = Color.FromArgb(workingImage[x, y], workingImage[x, y], workingImage[x, y]);
                        OutputImage.SetPixel(x, y, newColor); // set the pixel color at coordinate (x, y)
                    }
                }
            }

            pictureBox3.Image = (Image)OutputImage; // display output image
        }



        /*
         * applyProcessingFunction: defines behavior of function calls when "Apply" is pressed
         */
        private byte[,] applyProcessingFunction(byte[,] workingImage, byte[,] secondImage)
        {

            boundaryLabel.Visible = false;
            distinctValueLabel.Visible = false;
            peakValues.Visible = false;
            lineSegmentsLabel.Visible = false;

            byte[,] structureElement = createStructuringElement("square", structureElementSize);

            switch ((ProcessingFunctions)comboBox.SelectedIndex)
            {
                case ProcessingFunctions.Invert:
                    return invertImage(workingImage);
                case ProcessingFunctions.AdjustContrast:
                    return adjustContrast(workingImage);
                case ProcessingFunctions.ConvolutionFilter:
                    float[,] filter = createGaussianFilter(filterSize, filterSigma);
                    return convolveImage(workingImage, filter);
                case ProcessingFunctions.MedianFilter:
                    return medianFilter(workingImage, filterSize);
                case ProcessingFunctions.DetectEdges:
                    return edgeMagnitude(workingImage, horizontalKernel, verticalKernel);
                case ProcessingFunctions.Threshold:
                    return thresholdImage(workingImage, threshold);
                case ProcessingFunctions.EdgeSharpening:
                    return edgeSharpening(workingImage);
                case ProcessingFunctions.HistogramEqualization:
                    return histogramEqualization(workingImage);
                case ProcessingFunctions.dilateImage:
                    return dilateImage(workingImage, structureElement, isBinaryImage(workingImage));
                case ProcessingFunctions.erodeImage:
                    return erodeImage(workingImage, structureElement, isBinaryImage(workingImage));
                case ProcessingFunctions.openImage:
                    return openImage(workingImage, structureElement);
                case ProcessingFunctions.closeImage:
                    return closeImage(workingImage, structureElement);
                case ProcessingFunctions.andImages:
                    if (secondImage != null)
                    {
                        try
                        {
                            workingImage = thresholdImage(workingImage, threshold);
                            secondImage = thresholdImage(secondImage, threshold);
                            return andImages(workingImage, secondImage);
                        }
                        catch (ArgumentException ex)
                        {
                            MessageBox.Show($"AND operation failed: {ex.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                            return workingImage;
                        }
                    }
                    else
                    {
                        MessageBox.Show("Second image is required for AND operation. Please load another image.");
                        return workingImage;
                    }
                case ProcessingFunctions.orImages:
                    if (secondImage != null)
                    {
                        try
                        {
                            workingImage = thresholdImage(workingImage, threshold);
                            secondImage = thresholdImage(secondImage, threshold);
                            return orImages(workingImage, secondImage);
                        }
                        catch (ArgumentException ex)
                        {
                            MessageBox.Show($"OR operation failed: {ex.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                            return workingImage;
                        }
                    }
                    else
                    {
                        MessageBox.Show("Second image is required for OR operation. Please load another image.");
                        return workingImage;
                    }
                case ProcessingFunctions.largestShape:
                    return largestShape(workingImage);
                case ProcessingFunctions.valueCounting:
                    distinctValueLabel.Visible = true;
                    return valueCounting(workingImage);
                case ProcessingFunctions.traceBoundary:
                    // get the boundary image and the list of boundary points
                    (byte[,] boundaryImage, List<Point> boundaries) = traceBoundary(workingImage);

                    // show the boundaryLabel and display the boundary points
                    boundaryLabel.Visible = true;
                    boundaryLabel.Text = "Boundary Coordinates:\n";
                    foreach (Point point in boundaries)
                    {
                        boundaryLabel.Text += $"({point.X}, {point.Y}) \n";
                    }

                    // Return the boundary image
                    return boundaryImage;

                case ProcessingFunctions.complementImage:
                    return complementImage(pipelineOne(workingImage));

                case ProcessingFunctions.CountNonBackgroundValues:
                    distinctValueLabel.Visible = true;
                    return CountNonBackgroundValues(workingImage);








                case ProcessingFunctions.HoughTransform:

                    bool binary = isBinaryImage(workingImage);

                    return houghTransform(thresholdImage(workingImage, 150), 180, 200, binary);






                case ProcessingFunctions.peakFinding:

                    peakValues.Visible = true;

                    PeakFinding(workingImage, peakThreshold, closingSize);

                    return houghTransform(thresholdImage(workingImage, 150), 180, 200, isBinaryImage(workingImage));





                case ProcessingFunctions.houghTransformAngleLimits:

                    if (!TryParseAngleLimits()) return null;
                    bool binaryToken = isBinaryImage(workingImage);

                    return houghTransformAngleLimits(thresholdImage(workingImage, 150), 180, 300, binaryToken, lowerAngleLimit, upperAngleLimit);





                case ProcessingFunctions.houglineDetection:

                    lineSegmentsLabel.Visible = true;

                    byte[,] newImage = thresholdImage(workingImage, 150);
                    // Find (r, theta) pairs using the peak finding function
                    List<Tuple<int, int>> peaks = PeakFinding(newImage, peakThreshold, closingSize);



                    // Detect lines for each (r, theta)-pair found in the peak finding process
                    List<Tuple<Point, Point>> detectedLines = new List<Tuple<Point, Point>>();

                    foreach (var peak in peaks)
                    {
                        var lines = houghLineDetection(newImage, peak, minIntensityThreshold, minLength, maxGap);
                        detectedLines.AddRange(lines);
                    }

                    // Print detected lines in the label
                    PrintDetectedLines(detectedLines);

                    return workingImage;  // No image return for this case, only display lines





                case ProcessingFunctions.visualizeHoughLineSegments:
                    // Visualize detected Hough line segments

                    byte[,] thresholdedImageForVisualization = thresholdImage(workingImage, 150); // Threshold image for visualization

                    // Perform Peak Finding to get the (r, theta) pairs
                    List<Tuple<int, int>> detectedPeaks = PeakFinding(thresholdedImageForVisualization, peakThreshold, closingSize);

                    // Detect line segments for visualization
                    List<Tuple<Point, Point>> lineSegmentsForVisualization = new List<Tuple<Point, Point>>();
                    foreach (var peak in detectedPeaks)


                    {
                        // image, rThetaPair, minIntensityThreshold, minLength, maxGap
                        var lines = houghLineDetection(thresholdedImageForVisualization, peak, minIntensityThreshold, minLength, maxGap);
                        lineSegmentsForVisualization.AddRange(lines);
                    }

                    // Superimpose line segments on the original image
                    byte[,] visualizedImage = visualizeHoughLineSegments(workingImage, lineSegmentsForVisualization);

                    return visualizedImage;  // Return the visualized image with superimposed lines






                case ProcessingFunctions.houghTransformForCircles:


                    bool binaryToken2 = isBinaryImage(workingImage);

                    int maxRadius = 100; // Maximum radius for circles
                    int thresholdToken = 5; // Threshold for detecting the circles

                    byte[,] thresholdedImage = thresholdImage(workingImage, 150);


                    return houghTransformForCircles(thresholdedImage, maxRadius, thresholdToken);


                case ProcessingFunctions.preProcessing:
                    return preProcessing(workingImage);


                case ProcessingFunctions.detectStopSign:



                    return DetectStopSign(preProcessing(workingImage), thetaAxisSize: 180, rAxisSize: 200,
                        peakThreshold, closingSize);


                default:
                    return null;
            }
        }






        /*
         * saveButton_Click: process when user clicks "Save" button
         */
        private void saveButton_Click(object sender, EventArgs e)
        {
            if (OutputImage == null) return;                                // get out if no output image
            if (saveImageDialog.ShowDialog() == DialogResult.OK)
                OutputImage.Save(saveImageDialog.FileName);                 // save the output image
        }


        /*
         * convertToGrayScale: convert a three-channel color image to a single channel grayscale image
         * input:   inputImage          three-channel (Color) image
         * output:                      single-channel (byte) image
         */
        private byte[,] convertToGrayscale(Color[,] inputImage)
        {
            // create temporary grayscale image of the same size as input, with a single channel
            byte[,] tempImage = new byte[inputImage.GetLength(0), inputImage.GetLength(1)];

            // setup progress bar
            progressBar.Visible = true;
            progressBar.Minimum = 1;
            progressBar.Maximum = inputImage.GetLength(0) * inputImage.GetLength(1);
            progressBar.Value = 1;
            progressBar.Step = 1;

            // process all pixels in the image
            for (int x = 0; x < inputImage.GetLength(0); x++)               // loop over columns
                for (int y = 0; y < inputImage.GetLength(1); y++)           // loop over rows
                {
                    Color pixelColor = inputImage[x, y];                    // get pixel color
                    byte average = (byte)((pixelColor.R + pixelColor.B + pixelColor.G) / 3); // calculate average over the three channels
                    tempImage[x, y] = average;                              // set the new pixel color at coordinate (x,y)
                    progressBar.PerformStep();                              // increment progress bar
                }

            progressBar.Visible = false;                                    // hide progress bar

            return tempImage;
        }


        // ====================================================================
        // ============= YOUR FUNCTIONS FOR ASSIGNMENT 1 GO HERE ==============
        // ====================================================================

        // note: ensure functions have proper checks for size, channel and type

        /*
         * invertImage: invert a single channel (grayscale) image
         * input:   inputImage          single-channel (byte) image
         * output:                      single-channel (byte) image
         */
        private byte[,] invertImage(byte[,] inputImage)
        {
            // create temporary grayscale image
            byte[,] tempImage = new byte[inputImage.GetLength(0), inputImage.GetLength(1)];

            // TODO: add your functionality and checks
            for (int x = 0; x < inputImage.GetLength(0); x++)
            {
                for (int y = 0; y < inputImage.GetLength(1); y++)
                {
                    tempImage[x, y] = (byte)(255 - inputImage[x, y]); // invert the pixel value
                }
            }
            return tempImage;
        }


        /*
         * adjustContrast: create an image with the full range of intensity values used
         * input:   inputImage          single-channel (byte) image
         * output:                      single-channel (byte) image
         */
        private byte[,] adjustContrast(byte[,] inputImage)
        {
            // create temporary grayscale image
            byte[,] tempImage = new byte[inputImage.GetLength(0), inputImage.GetLength(1)];

            byte a_low = 255;
            byte a_high = 0;

            // find the current min and max intensity values
            for (int x = 0; x < inputImage.GetLength(0); x++)
            {
                for (int y = 0; y < inputImage.GetLength(1); y++)
                {
                    if (inputImage[x, y] < a_low)
                    {
                        a_low = inputImage[x, y];
                    }
                    if (inputImage[x, y] > a_high)
                    {
                        a_high = inputImage[x, y];
                    }
                }
            }

            // if image is uniform, nothing to stretch
            if (a_high == a_low)
                return tempImage;

            // apply the contrast adjustment formula
            for (int x = 0; x < inputImage.GetLength(0); x++)
            {
                for (int y = 0; y < inputImage.GetLength(1); y++)
                {
                    // normalize intensity values
                    tempImage[x, y] = (byte)((inputImage[x, y] - a_low) * 255 / (a_high - a_low));
                }
            }
            return tempImage;
        }


        /*
         * createGaussianFilter: create a Gaussian filter of specific square size and with a specified sigma
         * input:   size                length and width of the Gaussian filter (only odd sizes)
         *          sigma               standard deviation of the Gaussian distribution
         * output:                      Gaussian filter
         */


        private float[,] createGaussianFilter(byte size, float sigma)
        {
            // create temporary grayscale image

            float[,] filter = new float[size, size];


            // TODO: add your functionality and checks
            // check if size is odd and positive


            if (size % 2 == 0)
            {
                throw new ArgumentException("Only odd sizes accepted");
            }

            // calculate the center of the filter
            int center = size / 2;
            float sum = 0; // sum of filter values for normalization

            // calculate each value in the filter using the Gaussian formula
            for (int x = 0; x < size; x++)
            {
                for (int y = 0; y < size; y++)
                {
                    // coordinates relative to the center
                    int offsetX = x - center;
                    int offsetY = y - center;

                    // Guassian function calculation
                    int radius_squared = (offsetX * offsetX) + (offsetY * offsetY);
                    float sigma_squared = sigma * sigma;

                    float value = (float)(Math.Exp(-(radius_squared) / (2 * sigma_squared)));

                    filter[x, y] = value;

                    sum += value; // accumalate the sum for normalization
                }
            }

            // normalize the filter so the sum of all values equals 1
            for (int x = 0; x < size; x++)
            {
                for (int y = 0; y < size; y++)
                {
                    filter[x, y] /= sum;
                }
            }

            return filter;
        }



        /*
         * convolveImage: apply linear filtering of an input image
         * input:   inputImage          single-channel (byte) image
         *          filter              linear kernel
         * output:                      single-channel (byte) image
         */
        private byte[,] convolveImage(byte[,] inputImage, float[,] filter)
        {
            // create temporary grayscale image
            byte[,] tempImage = new byte[inputImage.GetLength(0), inputImage.GetLength(1)];

            // TODO: add your functionality and checks, think about border handling and type conversion

            int filterSize = filter.GetLength(0);

            // pad the input image to handle borders
            int paddedHeight = inputImage.GetLength(0) + filterSize;
            int paddedWidth = inputImage.GetLength(1) + filterSize;
            byte[,] paddedImage = new byte[paddedHeight, paddedWidth];

            // copy input image into the center of the padded image
            for (int i = 0; i < inputImage.GetLength(0); i++)
            {
                for (int j = 0; j < inputImage.GetLength(1); j++)
                {
                    paddedImage[i + (filterSize / 2), j + (filterSize / 2)] = inputImage[i, j];
                }
            }

            // apply the convolution
            for (int i = 0; i < inputImage.GetLength(0); i++)
            {
                for (int j = 0; j < inputImage.GetLength(1); j++)
                {
                    float sum = 0.0f;

                    // Apply the filter to the neighborhood
                    for (int fx = 0; fx < filterSize; fx++)
                    {
                        for (int fy = 0; fy < filterSize; fy++)
                        {
                            int imageX = i + fx;
                            int imageY = j + fy;

                            sum += paddedImage[imageX, imageY] * filter[fx, fy];
                        }
                    }

                    // convert sum to byte value, clipping to the range [0,255]
                    int pixelValue = (int)Math.Round(sum); // round sum to nearest integer

                    if (pixelValue < 0)
                    {
                        pixelValue = 0;
                    }
                    else if (pixelValue > 255)
                    {
                        pixelValue = 255;
                    }

                    tempImage[i, j] = (byte)(pixelValue);
                }
            }
            return tempImage;
        }


        /*
         * medianFilter: apply median filtering on an input image with a kernel of specified size
         * input:   inputImage          single-channel (byte) image
         *          size                length/width of the median filter kernel
         * output:                      single-channel (byte) image
         */
        private byte[,] medianFilter(byte[,] inputImage, byte size)
        {
            // create temporary grayscale image
            byte[,] tempImage = new byte[inputImage.GetLength(0), inputImage.GetLength(1)];

            // TODO: add your functionality and checks, think about border handling and type conversion


            if (size % 2 == 0)
            {
                throw new ArgumentException("Only odd sizes accepted");
            }

            // pad the input image to handle borders
            int paddedHeight = inputImage.GetLength(0) + size;
            int paddedWidth = inputImage.GetLength(1) + size;
            byte[,] paddedImage = new byte[paddedHeight, paddedWidth];

            // Copy the input image into the center of the padded image
            for (int i = 0; i < inputImage.GetLength(0); i++)
            {
                for (int j = 0; j < inputImage.GetLength(1); j++)
                {
                    paddedImage[i + size / 2, j + size / 2] = inputImage[i, j];
                }
            }

            // Traverse each pixel in the original image
            for (int x = 0; x < inputImage.GetLength(0); x++)
            {
                for (int y = 0; y < inputImage.GetLength(1); y++)
                {
                    // Create a list to hold neighborhood pixel values
                    List<byte> neighborhood = new List<byte>();

                    // Collect values from the neighborhood defined by filter size
                    for (int i = 0; i < size; i++)
                    {
                        for (int j = 0; j < size; j++)
                        {
                            // Access the corresponding padded pixel
                            neighborhood.Add(paddedImage[x + i, y + j]);
                        }
                    }

                    // Sort the neighborhood values and find the median
                    neighborhood.Sort();
                    byte medianValue = neighborhood[neighborhood.Count / 2];

                    // Set the median value to the current pixel in the output image
                    tempImage[x, y] = medianValue;
                }
            }


            return tempImage;
        }


        /*
         * edgeMagnitude: calculate the image derivative of an input image and a provided edge kernel
         * input:   inputImage          single-channel (byte) image
         *          horizontalKernel    horizontal edge kernel
         *          virticalKernel      vertical edge kernel
         * output:                      single-channel (byte) image
         */
        private byte[,] edgeMagnitude(byte[,] inputImage, sbyte[,] horizontalKernel, sbyte[,] verticalKernel)
        {
            // create temporary grayscale image
            byte[,] tempImage = new byte[inputImage.GetLength(0), inputImage.GetLength(1)];

            // Check for null objects
            if (inputImage == null || horizontalKernel == null || verticalKernel == null)
            {
                throw new ArgumentNullException("Input image or kernels are null.");
            }

            // get kernel dimensions
            int kernelSize = horizontalKernel.GetLength(0);
            int halfKernel = kernelSize / 2;

            for (int x = 0; x < inputImage.GetLength(0); x++)
            {
                for (int y = 0; y < inputImage.GetLength(1); y++)
                {
                    int gx = 0;
                    int gy = 0;

                    // convolution apply kernels
                    for (int i = 0; i < kernelSize; i++)
                    {
                        for (int j = 0; j < kernelSize; j++)
                        {
                            // Calculate the offset from the center of the kernel
                            int px = Math.Min(Math.Max(x + i - halfKernel, 0), inputImage.GetLength(0) - 1);
                            int py = Math.Min(Math.Max(y + j - halfKernel, 0), inputImage.GetLength(1) - 1);

                            byte pixel = inputImage[px, py];

                            gx += pixel * horizontalKernel[i, j];
                            gy += pixel * verticalKernel[i, j];
                        }
                    }

                    // calculate the magnitude
                    int magnitude = (int)Math.Sqrt(gx * gx + gy * gy);

                    // ensure the magnitude is within the range [0, 255]
                    tempImage[x, y] = (byte)Math.Min(255, Math.Max(0, magnitude));
                }
            }

            return tempImage;
        }



        /*
         * thresholdImage: threshold a grayscale image
         * input:   inputImage          single-channel (byte) image
         * output:                      single-channel (byte) image with on/off values
         */
        private byte[,] thresholdImage(byte[,] inputImage, byte threshold)
        {
            // create temporary grayscale image
            byte[,] tempImage = new byte[inputImage.GetLength(0), inputImage.GetLength(1)];

            // TODO: add your functionality and checks, think about how to represent the binary values

            for (int x = 0; x < inputImage.GetLength(0); x++)
            {
                for (int y = 0; y < inputImage.GetLength(1); y++)
                {
                    if (inputImage[x, y] >= threshold)
                    {
                        tempImage[x, y] = 255;
                    }
                    else
                    {
                        tempImage[x, y] = 0;
                    }
                }
            }

            return tempImage;
        }

        // bonus marks: edgeSharpening & histogramEqualization implementations

        /*
         * edge sharpening:  sharpening the edge of the image 
         * input:   inputImage          single-channel (byte) image
         * output:                      single-channel (byte) image 
         */
        private byte[,] edgeSharpening(byte[,] inputImage)
        {
            // create temporary grayscale image
            byte[,] tempImage = new byte[inputImage.GetLength(0), inputImage.GetLength(1)];

            // apply laplacian kernel
            float[,] laplacianKernel = {{ 0, -1, 0 },
                                        { -1, 4, -1 },
                                        { 0, -1, 0 }};

            // apply the Laplacian filter (edge detection) to the input image
            byte[,] edges = convolveImage(inputImage, laplacianKernel);

            // define weight for edge sharpening
            float w = 0.9f;

            // Combine the original image with the filter for sharpening
            for (int x = 0; x < inputImage.GetLength(0); x++)
            {
                for (int y = 0; y < inputImage.GetLength(1); y++)
                {
                    // get the sharpened value by adding edge detail to the original image
                    int sharpenedValue = (int)(inputImage[x, y] + w * edges[x, y]);

                    // Ensure that the pixel value range between 0 and 255
                    if (sharpenedValue < 0)
                    {
                        sharpenedValue = 0;
                    }
                    else if (sharpenedValue > 255)
                    {
                        sharpenedValue = 255;
                    }

                    tempImage[x, y] = (byte)sharpenedValue;
                }
            }

            return tempImage;
        }

        private byte[,] histogramEqualization(byte[,] inputImage)
        {
            // create temporary grayscale image
            byte[,] tempImage = new byte[inputImage.GetLength(0), inputImage.GetLength(1)];

            // calculate the histogram, count the number of each pixel value
            int[] histogram = new int[256];
            for (int x = 0; x < inputImage.GetLength(0); x++)
            {
                for (int y = 0; y < inputImage.GetLength(1); y++)
                {
                    histogram[inputImage[x, y]]++;
                }
            }

            // calculate the cumulative distribution function (CDF)
            // count the number of pixel below the value
            int[] cdf = new int[256];
            cdf[0] = histogram[0];

            for (int i = 1; i < 256; i++)
            {
                cdf[i] = cdf[i - 1] + histogram[i];
            }

            // Normalize the CDF to map the pixel values between [0, 255]
            int total = inputImage.GetLength(0) * inputImage.GetLength(1);
            byte[] p_equalization = new byte[256];

            for (int i = 0; i < 256; i++)
            {
                p_equalization[i] = (byte)((cdf[i]) * 255 / (total));
            }

            // map the pixel values in the input image
            for (int x = 0; x < inputImage.GetLength(0); x++)
            {
                for (int y = 0; y < inputImage.GetLength(1); y++)
                {
                    tempImage[x, y] = p_equalization[inputImage[x, y]];
                }
            }

            return tempImage;
        }

        // creating pipeline 1
        private byte[,] pipelineOne(byte[,] inputImage)
        {

            // sobel edge detectors for gx and gy
            sbyte[,] horizontalKernel = new sbyte[,]
            {
                        {1, 2, 1},
                        {0, 0, 0},
                        {-1, -2, -1}
            };
            sbyte[,] verticalKernel = new sbyte[,]
            {
                        {-1, 0, 1},
                        {-2, 0, 2},
                        {-1, 0, 1}
            };

            // image A: converting to grayscale and adjusting the contrast
            byte[,] imageA = adjustContrast(inputImage);

            // applying the Gaussian filter
            float[,] filter = createGaussianFilter(5, 3f);
            byte[,] newGaussian = convolveImage(imageA, filter);

            // applying edge detection
            byte[,] newEdgeDetection = edgeMagnitude(newGaussian, horizontalKernel, verticalKernel);

            // threshold the image
            // threshold is adjustable: currently threshold = 30;
            byte[,] newThreshold = thresholdImage(newEdgeDetection, 30);

            return newThreshold;
        }

        private byte[,] pipelineTwo(byte[,] inputImage)
        {

            // sobel edge detectors for gx and gy
            sbyte[,] horizontalKernel = new sbyte[,]
            {
                        {1, 2, 1},
                        {0, 0, 0},
                        {-1, -2, -1}
            };
            sbyte[,] verticalKernel = new sbyte[,]
            {
                        {-1, 0, 1},
                        {-2, 0, 2},
                        {-1, 0, 1}
            };

            // Image A: converting to grayscale and adjusting the contrast
            byte[,] imageA = adjustContrast(inputImage);

            // applying the median filter
            byte[,] newMedian = medianFilter(imageA, 5);

            // applying edge detection
            byte[,] newEdgeDetection = edgeMagnitude(newMedian, horizontalKernel, verticalKernel);

            // threshold the image
            // threshold is adjustable: currently threshold = 40;
            byte[,] newThreshold = thresholdImage(newEdgeDetection, 40);

            return newThreshold;
        }


        // ====================================================================
        // ============= YOUR FUNCTIONS FOR ASSIGNMENT 2 GO HERE ==============
        // ====================================================================

        /*
         * createStructuringElement: generates a structuring element based on a given size and shape.
         * input: shape (string)     "plus" or "square"
         *        size (int)         the size of the structuring element (odd number)
         * output: 2D binary structuring element (byte array)
         */
        private byte[,] createStructuringElement(string shape, int size)
        {
            // check if the size is odd
            if (size % 2 == 0)
            {
                throw new ArgumentException("Size must be an odd number");
            }

            byte[,] structuringElement;

            if (shape == "square")
            {
                structuringElement = new byte[size, size];

                // set all values to 1 (filled square)

                for (int i = 0; i < size; i++)
                {
                    for (int j = 0; j < size; j++)
                    {
                        structuringElement[i, j] = 1;
                    }
                }
            }
            else if (shape == "plus")
            {
                // start with a 3x3 plus-shaped element
                structuringElement = new byte[3, 3]
                {
                    { 0, 1, 0},
                    { 1, 1, 1},
                    { 0, 1, 0}
                };

                // iteratively dilate to achieve the desired size
                while (structuringElement.GetLength(0) < size)
                {
                    structuringElement = dilateImage(structuringElement, new byte[3, 3]
                    {
                        {0, 1, 0},
                        {1, 1, 1},
                        {0, 1, 0}
                    }, true); // true because it is a 2D binary structuring element
                }
            }
            else
            {
                throw new ArgumentException("Invalid shape. Must be either plus or square");
            }

            return structuringElement;
        }

        /*
         * dilateImage: Perform dilation of an input image using the provided structuring element.
         * input:      inputImage (byte[,]) single-channel grayscale or binary image
         *             structuringElement (byte[,]) structuring element for dilation
         *             isBinary (bool) Flag to indicate if the image is binary (true) or grayscale (false)
         * output:     Dilated image (byte array)
         */
        private byte[,] dilateImage(byte[,] inputImage, byte[,] structuringElement, bool isBinary)
        {
            int imageHeight = inputImage.GetLength(0);
            int imageWidth = inputImage.GetLength(1);
            int seHeight = structuringElement.GetLength(0);
            int seWidth = structuringElement.GetLength(1);

            // Create an output image (same size as input)
            byte[,] outputImage = new byte[imageHeight, imageWidth];

            // Calculate structuring element center
            int seCenterX = seHeight / 2;
            int seCenterY = seWidth / 2;

            // Iterate over the input image
            for (int x = 0; x < imageHeight; x++)
            {
                for (int y = 0; y < imageWidth; y++)
                {
                    // For binary images, start with 0 (black). For grayscale, start with minimum value (0).
                    byte maxValue = (byte)(isBinary ? 0 : 0);

                    // Iterate over the structuring element
                    for (int i = 0; i < seHeight; i++)
                    {
                        for (int j = 0; j < seWidth; j++)
                        {
                            int offsetX = x + i - seCenterX;
                            int offsetY = y + j - seCenterY;

                            // Check if the structuring element is within bounds of the input image
                            if (offsetX >= 0 && offsetX < imageHeight && offsetY >= 0 && offsetY < imageWidth)
                            {
                                if (structuringElement[i, j] == 1) // Only apply dilation where the structuring element has 1
                                {
                                    if (isBinary)
                                    {
                                        // For binary images, if any pixel is greater than 0, the current pixel becomes 255 (white)
                                        if (inputImage[offsetX, offsetY] > 0)
                                        {
                                            maxValue = 255; // Use 255 for binary white (instead of 1)
                                            break; // No need to check further, since binary dilation will make it 255
                                        }
                                    }
                                    else
                                    {
                                        // For grayscale, we are finding the maximum value in the neighborhood
                                        maxValue = Math.Max(maxValue, inputImage[offsetX, offsetY]);
                                    }
                                }
                            }
                        }

                        // For binary, if we've already set maxValue to 255, we can stop checking further.
                        if (isBinary && maxValue == 255)
                        {
                            break;
                        }
                    }

                    // Set the pixel value in the output image
                    outputImage[x, y] = maxValue;
                }
            }

            return outputImage;
        }



        /*
         * erodeImage: Perform erosion of an input image using the provided structuring element.
         * input:   inputImage (byte[,]) single-channel grayscale or binary image
         *          structuringElement (byte[,]) structuring element for erosion
         *          isBinary (bool) Flag to indicate if the image is binary (true) or grayscale (false)
         * output:  Eroded image (byte array)
         */
        private byte[,] erodeImage(byte[,] inputImage, byte[,] structuringElement, bool isBinary)
        {
            int imageHeight = inputImage.GetLength(0);
            int imageWidth = inputImage.GetLength(1);
            int seHeight = structuringElement.GetLength(0);
            int seWidth = structuringElement.GetLength(1);

            // Create an output image (same size as input)
            byte[,] outputImage = new byte[imageHeight, imageWidth];

            // Calculate structuring element center
            int seCenterX = seHeight / 2;
            int seCenterY = seWidth / 2;

            // Iterate over the input image
            for (int x = 0; x < imageHeight; x++)
            {
                for (int y = 0; y < imageWidth; y++)
                {
                    // For binary images, start with 255 (white). For grayscale, start with maximum value (255).
                    byte minValue = (byte)(isBinary ? 255 : 255);

                    // Iterate over the structuring element
                    for (int i = 0; i < seHeight; i++)
                    {
                        for (int j = 0; j < seWidth; j++)
                        {
                            int offsetX = x + i - seCenterX;
                            int offsetY = y + j - seCenterY;

                            // Check if the structuring element is within bounds of the input image
                            if (offsetX >= 0 && offsetX < imageHeight && offsetY >= 0 && offsetY < imageWidth)
                            {
                                if (structuringElement[i, j] == 1) // Only apply erosion where the structuring element has 1
                                {
                                    if (isBinary)
                                    {
                                        // For binary images, if any pixel is 0, the current pixel becomes 0
                                        if (inputImage[offsetX, offsetY] == 0)
                                        {
                                            minValue = 0;
                                            break; // No need to check further, since binary erosion will make it 0
                                        }
                                    }
                                    else
                                    {
                                        // For grayscale, we are finding the minimum value in the neighborhood
                                        minValue = Math.Min(minValue, inputImage[offsetX, offsetY]);
                                    }
                                }
                            }
                        }

                        // For binary, if we've already set minValue to 0, we can stop checking further.
                        if (isBinary && minValue == 0)
                        {
                            break;
                        }
                    }

                    // Set the pixel value in the output image
                    outputImage[x, y] = minValue;
                }
            }

            return outputImage;
        }


        /*
         * isBinaryImage: Determines if the input image is binary or grayscale.
         * input:    inputImage (byte[,]) single-channel image (byte array)
         * output:   Returns true if the image is binary, false if it is grayscale.
         */
        private bool isBinaryImage(byte[,] inputImage)
        {
            HashSet<byte> uniqueValues = new HashSet<byte>();

            int imageHeight = inputImage.GetLength(0);
            int imageWidth = inputImage.GetLength(1);

            // Iterate over the image to find unique values
            for (int x = 0; x < imageHeight; x++)
            {
                for (int y = 0; y < imageWidth; y++)
                {
                    uniqueValues.Add(inputImage[x, y]);

                    // If more than two unique values are found, it's not binary
                    if (uniqueValues.Count > 2)
                    {
                        return false; // Grayscale
                    }
                }
            }

            // If the unique values set contains exactly 2 values (e.g., 0 and 255), it's binary
            return (uniqueValues.Count == 2);
        }


        /*
         * openImage: Perform morphological opening (erosion followed by dilation).
         * input:    inputImage (byte[,]) single-channel grayscale or binary image
         *           structuringElement (byte[,]) structuring element for the operation
         * output:   Image after the morphological opening
         */
        private byte[,] openImage(byte[,] inputImage, byte[,] structuringElement)
        {
            bool isBinary = isBinaryImage(inputImage);

            // Perform erosion followed by dilation
            byte[,] erodedImage = erodeImage(inputImage, structuringElement, isBinary);
            byte[,] openedImage = dilateImage(erodedImage, structuringElement, isBinary);

            return openedImage;
        }

        /*
         * closeImage: Perform morphological closing (dilation followed by erosion).
         * input:    inputImage (byte[,]) single-channel grayscale or binary image
         *           structuringElement (byte[,]) structuring element for the operation
         * output:   Image after the morphological closing
         */
        private byte[,] closeImage(byte[,] inputImage, byte[,] structuringElement)
        {
            bool isBinary = isBinaryImage(inputImage);

            // Perform dilation followed by erosion
            byte[,] dilatedImage = dilateImage(inputImage, structuringElement, isBinary);
            byte[,] closedImage = erodeImage(dilatedImage, structuringElement, isBinary);

            return closedImage;
        }

        /*
         * checkDimensions: Verifies that two images have the same dimensions.
         * input:    image1, image2 (byte[,]) two images to compare
         * output:   Returns true if the dimensions match, false otherwise.
         */
        private bool checkDimensions(byte[,] image1, byte[,] image2)
        {
            return (image1.GetLength(0) == image2.GetLength(0)) && (image1.GetLength(1) == image2.GetLength(1));
        }

        /*
         * andImages: Perform pixel-wise AND operation on two binary images.
         * input:    image1 (byte[,]) first binary image
         *           image2 (byte[,]) second binary image
         * output:   Resulting binary image after AND operation
         */
        private byte[,] andImages(byte[,] image1, byte[,] image2)
        {
            // Check if both images are binary
            if (!isBinaryImage(image1) || !isBinaryImage(image2))
            {
                throw new ArgumentException("Both images must be binary.");
            }

            // Check if both images have the same dimensions
            if (!checkDimensions(image1, image2))
            {
                throw new ArgumentException("Images must have the same dimensions.");
            }

            int imageHeight = image1.GetLength(0);
            int imageWidth = image1.GetLength(1);

            // Create output image for AND operation
            byte[,] outputImage = new byte[imageHeight, imageWidth];

            // Perform pixel-wise AND operation
            for (int x = 0; x < imageHeight; x++)
            {
                for (int y = 0; y < imageWidth; y++)
                {
                    outputImage[x, y] = (byte)(image1[x, y] & image2[x, y]);
                }
            }

            return outputImage;
        }


        /*
         * orImages: Perform pixel-wise OR operation on two binary images.
         * input:    image1 (byte[,]) first binary image
         *           image2 (byte[,]) second binary image
         * output:   Resulting binary image after OR operation
         */
        private byte[,] orImages(byte[,] image1, byte[,] image2)
        {
            if (!isBinaryImage(image1) || !isBinaryImage(image2))
            {
                throw new ArgumentException("Both images must be binary.");
            }

            // Check if both images have the same dimensions
            if (!checkDimensions(image1, image2))
            {
                throw new ArgumentException("Images must have the same dimensions.");
            }

            int imageHeight = image1.GetLength(0);
            int imageWidth = image1.GetLength(1);

            // Create output image for OR operation
            byte[,] outputImage = new byte[imageHeight, imageWidth];

            // Perform pixel-wise OR operation
            for (int x = 0; x < imageHeight; x++)
            {
                for (int y = 0; y < imageWidth; y++)
                {
                    outputImage[x, y] = (byte)(image1[x, y] | image2[x, y]);
                }
            }

            return outputImage;
        }

        /*
         * valueCounting: count the distinct value and generate the value occurance histogram
         * input:   inputImage          single-channel (byte) image
         * output:  histogram           histogram how often each value occurs
         *          
         */
        public byte[,] valueCounting(byte[,] inputImage)
        {
            // empty array to store the count of value [0,255]
            int[] histogram = new int[256];
            int distinctValues = 0;

            // track distinct values
            for (int x = 0; x < inputImage.GetLength(0); x++)
            {
                for (int y = 0; y < inputImage.GetLength(1); y++)
                {
                    int value = inputImage[x, y];
                    // is the value has not present yet, increase the distinct Count
                    if (histogram[value] == 0)
                    {
                        distinctValues++;
                    }

                    // increase the value count
                    histogram[value]++;
                }
            }

            // Histogram image with additional row at index 0 for distinct values indicator
            byte[,] histogramImage = new byte[256, 101];
            int maxCount = histogram.Max();
            // avoid maxCount = 0
            if (maxCount == 0)
                maxCount = 1;

            //  distinct values in the top row
            for (int i = 0; i < 256; i++)
            {
                if (histogram[i] != 0)
                {
                    histogramImage[i, 0] = 255;
                }
            }

            distinctValueLabel.Text = $"Distinct Values: {distinctValues}";

            // Create histogram
            for (int i = 0; i < 256; i++)
            {
                // Normalize to 100 pixels high
                int height = (histogram[i] * 100) / maxCount;
                for (int j = 100; j > 100 - height; j--)
                {
                    // fill up the histogram with white
                    histogramImage[i, j] = 255;
                }
            }

            return (histogramImage);
        }

        /*
         * traceBoundary: traces the outer boundary of the first foreground object in a binary image
         * input:   inputImage          single-channel (byte) binary image (0 for background, 255 for foreground)
         * output:  outputImage         single-channel (byte) binary image (0 for background, 255 for boundary of forground)
         *          boundaryList        list of (x, y) Points representing the boundary
         */
        private (byte[,], List<Point>) traceBoundary(byte[,] inputImage)
        {
            List<Point> boundaryPoints = new List<Point>();

            byte[,] tempImage = new byte[inputImage.GetLength(0), inputImage.GetLength(1)];

            // Fill the temporary image with black (background)
            for (int x = 0; x < inputImage.GetLength(0); x++)
            {
                for (int y = 0; y < inputImage.GetLength(1); y++)
                {
                    tempImage[x, y] = 0;
                }
            }

            // Directions in Moore Neighborhood (8 directions)
            Point[] directions = new Point[]
            {
                new Point(0, -1),  // North
                new Point(1, -1),  // North-East
                new Point(1, 0),   // East
                new Point(1, 1),   // South-East
                new Point(0, 1),   // South
                new Point(-1, 1),  // South-West
                new Point(-1, 0),  // West
                new Point(-1, -1)  // North-West
            };

            // Find the first foreground (white) pixel
            // assume the input image is not black, there must be a white point exist
            Point start = Point.Empty;
            for (int x = 0; x < inputImage.GetLength(0); x++)
            {
                for (int y = 0; y < inputImage.GetLength(1); y++)
                {
                    if (inputImage[x, y] == 255)
                    {
                        start = new Point(x, y);
                        break;
                    }
                }
                // Stop after finding the starting point
                if (start != Point.Empty) break;
            }

            // Start boundary tracing
            Point current = start;
            Point previous = new Point(start.X, start.Y - 1);

            do
            {
                // Add the current boundary point to the list
                boundaryPoints.Add(current);
                // mark the boundary in the output image
                tempImage[current.X, current.Y] = 255;

                // Find the index of the direction from which we came
                int dirIndex = -1;
                for (int i = 0; i < directions.Length; i++)
                {
                    if (previous.X == current.X + directions[i].X
                        && previous.Y == current.Y + directions[i].Y)
                    {
                        dirIndex = i;
                        break;
                    }
                }

                // Check all directions starting from the next direction
                bool foundNext = false;
                for (int i = 0; i < 8; i++)
                {
                    int nextDirIndex = (dirIndex + 1 + i) % 8;
                    Point nextDir = directions[nextDirIndex];
                    Point nextPoint = new Point(current.X + nextDir.X, current.Y + nextDir.Y);

                    // Ensure next point is still within image boundary
                    if (nextPoint.X >= 0 && nextPoint.X < inputImage.GetLength(0)
                        && nextPoint.Y >= 0 && nextPoint.Y < inputImage.GetLength(1))
                    {
                        if (inputImage[nextPoint.X, nextPoint.Y] == 255)
                        {
                            previous = current;
                            current = nextPoint;
                            foundNext = true;
                            break;
                        }
                    }
                }

                // If no neighboring boundary pixel is found, break the loop
                if (!foundNext) break;

            } while (current != start);  // Stop when looped back to the start

            // Return the marked boundary image and points
            return (tempImage, boundaryPoints);
        }

        /*
         * largestShape: Identify and retain only the largest connected shape in a binary image
         * input:   inputImage          single-channel (byte) binary image
         * output:  largestShape   single-channel (byte) image with only the largest shape
         */
        private byte[,] largestShape(byte[,] inputImage)
        {
            int width = inputImage.GetLength(0);
            int height = inputImage.GetLength(1);

            // since the shape is white, the connected area have the largest pixel value then it will be the largest shape
            // Create label array to store labels for connected components
            int[,] labeledImage = new int[width, height];
            int currentLabel = 1;

            //  Label all connected components
            Dictionary<int, int> labelCounts = new Dictionary<int, int>();

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    // If pixel is foreground and not yet labeled
                    if (inputImage[x, y] == 255 && labeledImage[x, y] == 0)
                    {
                        int pixelCount = LabelComponent(inputImage, labeledImage, x, y, currentLabel);

                        // Store the count of pixels for this label
                        labelCounts[currentLabel] = pixelCount;
                        currentLabel++;
                    }
                }
            }

            // find the label with the highest pixel count
            int largestLabel = 0;
            int maxSize = 0;

            foreach (var kvp in labelCounts)
            {
                if (kvp.Value > maxSize)
                {
                    largestLabel = kvp.Key;
                    maxSize = kvp.Value;
                }
            }

            // create a new image and retain only the largest shape
            byte[,] largestShapeImage = new byte[width, height];

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    // Set pixel to 255 (white) if it's part of the largest shape, otherwise 0 (black)
                    if (labeledImage[x, y] == largestLabel)
                    {
                        largestShapeImage[x, y] = 255;
                    }
                    else
                    {
                        largestShapeImage[x, y] = 0;
                    }
                }
            }

            return largestShapeImage;
        }

        // count the total pixel value of the connected shape
        private int LabelComponent(byte[,] inputImage, int[,] labeledImage, int startX, int startY, int label)
        {
            int width = inputImage.GetLength(0);
            int height = inputImage.GetLength(1);
            int count = 0;

            // Directions of 8 neighbors
            int[] dx = { -1, 1, 0, 0, -1, -1, 1, 1 };
            int[] dy = { 0, 0, -1, 1, -1, 1, -1, 1 };

            // check the value and push in stack
            Stack<(int, int)> stack = new Stack<(int, int)>();
            stack.Push((startX, startY));
            labeledImage[startX, startY] = label;

            while (stack.Count > 0)
            {
                (int x, int y) = stack.Pop();
                count++;

                // Check all 8 neighbors
                for (int i = 0; i < 8; i++)
                {
                    int newX = x + dx[i];
                    int newY = y + dy[i];

                    // Check if within bounds and if it’s part of the same component
                    if (newX >= 0 && newX < width
                        && newY >= 0 && newY < height
                        && inputImage[newX, newY] == 255 && labeledImage[newX, newY] == 0)
                    {
                        labeledImage[newX, newY] = label;
                        stack.Push((newX, newY));
                    }
                }
            }

            return count;
        }


        private byte[,] complementImage(byte[,] binaryImage)
        {
            if (!isBinaryImage(binaryImage))
            {
                throw new ArgumentException("The input image must be binary.");
            }

            int imageHeight = binaryImage.GetLength(0);
            int imageWidth = binaryImage.GetLength(1);

            // Create output image for the complement operation
            byte[,] outputImage = new byte[imageHeight, imageWidth];

            // Perform pixel-wise complement operation
            for (int x = 0; x < imageHeight; x++)
            {
                for (int y = 0; y < imageWidth; y++)
                {
                    outputImage[x, y] = (byte)(binaryImage[x, y] == 0 ? 255 : 0);
                }
            }

            return outputImage;
        }

        private byte[,] CountNonBackgroundValues(byte[,] inputImage)
        {
            byte[,] binaryImage = thresholdImage(inputImage, 127);
            // here to modifythe structuring element
            byte[,] structuringElement = createStructuringElement("square", 23);
            byte[,] imageG = openImage(binaryImage, structuringElement);

            // background color is black
            byte backgroundValue = 0;
            int nonBackgroundCount = 0;

            for (int x = 0; x < imageG.GetLength(0); x++)
            {
                for (int y = 0; y < imageG.GetLength(1); y++)
                {
                    // if the pixel is not background
                    if (imageG[x, y] != backgroundValue)
                    {
                        nonBackgroundCount++;
                    }
                }
            }

            distinctValueLabel.Text = $"Number of non-background values: {nonBackgroundCount}";

            return imageG;
        }


        // ====================================================================
        // ============= YOUR FUNCTIONS FOR ASSIGNMENT 3 GO HERE ==============
        // ====================================================================


        private byte[,] houghTransform(byte[,] inputData, int thetaAxisSize, int rAxisSize, bool isBinary)
        {
            int width = inputData.GetLength(1);
            int height = inputData.GetLength(0);
            int maxRadius = (int)Math.Ceiling(Math.Sqrt(width * width + height * height));
            int halfRAxisSize = rAxisSize >> 1; // Bit shift to divide by 2
            int[,] accumulator = new int[thetaAxisSize, rAxisSize]; // Use int for the accumulator initially

            // Precompute sin and cos tables for theta values
            double[] sinTable = new double[thetaAxisSize];
            double[] cosTable = new double[thetaAxisSize];
            for (int theta = 0; theta < thetaAxisSize; theta++)
            {
                double thetaRadians = theta * Math.PI / thetaAxisSize;
                sinTable[theta] = Math.Sin(thetaRadians);
                cosTable[theta] = Math.Cos(thetaRadians);
            }

            // Apply Hough Transform on the entire image
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    byte pixelValue = inputData[y, x];

                    if (pixelValue > 0) // Only process non-zero pixels
                    {
                        for (int theta = 0; theta < thetaAxisSize; theta++)
                        {
                            double r = cosTable[theta] * x + sinTable[theta] * y;
                            int rScaled = (int)Math.Round(r * halfRAxisSize / maxRadius) + halfRAxisSize;

                            if (rScaled >= 0 && rScaled < rAxisSize) // Ensure it's within bounds
                            {
                                // Increment the accumulator differently based on whether it's binary or grayscale
                                if (isBinary)
                                {
                                    accumulator[theta, rScaled]++; // For binary, increment by 1
                                }
                                else
                                {
                                    accumulator[theta, rScaled] += pixelValue; // For grayscale, increment by pixel value
                                }
                            }
                        }
                    }
                }
            }

            // Normalize the accumulator to byte[,] for output
            byte[,] outputData = NormalizeAccumulator(accumulator, thetaAxisSize, rAxisSize);

            return outputData;
        }


        // Hough Transform specifically for circles
        private byte[,] houghTransformForCircles(byte[,] edgeImage, int maxRadius, int threshold)
        {
            int width = edgeImage.GetLength(0);
            int height = edgeImage.GetLength(1);


            // Initialize the accumulator
            int[,,] accumulator = new int[width, height, maxRadius + 1];

            // Loop through each edge pixel in the edge image
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    // Check if the current pixel is an edge pixel (assumed binary edge image)
                    if (edgeImage[x, y] > 0) // Assuming non-zero value indicates edge
                    {
                        // For each radius
                        for (int r = 0; r <= maxRadius; r++)
                        {
                            // For each angle in degrees
                            for (int theta = 0; theta < 360; theta++)
                            {
                                // Convert angle to radians
                                double thetaRad = theta * Math.PI / 180.0;
                                int a = (int)(x - r * Math.Cos(thetaRad));
                                int b = (int)(y - r * Math.Sin(thetaRad));

                                // Check if (a,b) is within the bounds of the accumulator
                                if (a >= 0 && a < width && b >= 0 && b < height)
                                {
                                    accumulator[a, b, r]++;
                                }
                            }
                        }
                    }
                }
            }

            // Create a byte array for the result (accumulator)
            byte[,] result = new byte[width, height];

            // Normalize the accumulator based on the threshold
            for (int a = 0; a < width; a++)
            {
                for (int b = 0; b < height; b++)
                {
                    // Find the maximum votes for each (a,b) pair
                    for (int r = 0; r <= maxRadius; r++)
                    {
                        if (accumulator[a, b, r] > threshold)
                        {
                            result[a, b] = (byte)Math.Min(accumulator[a, b, r], 255); // Clamp to byte range
                        }
                    }
                }
            }

            return result;
        }



        private byte[,] NormalizeAccumulator(int[,] accumulator, int thetaAxisSize, int rAxisSize)
        {
            int maxAccValue = accumulator.Cast<int>().Max();
            byte[,] outputData = new byte[thetaAxisSize, rAxisSize];

            if (maxAccValue == 0) return outputData; // nothing to normalize

            for (int theta = 0; theta < thetaAxisSize; theta++)
            {
                for (int r = 0; r < rAxisSize; r++)
                {
                    outputData[theta, r] = (byte)Math.Min(accumulator[theta, r] * 255 / maxAccValue, 255); // Scale to byte range
                }
            }

            return outputData;
        }




        private List<Tuple<int, int>> PeakFinding(byte[,] image, int peakThreshold, int closingSize)
        {

            bool binary = isBinaryImage(image);
            // Step 1: Hough Transform
            byte[,] houghSpace = houghTransform(image, 180, 200, binary);

            // Step 2: Thresholding
            int thetaCount = houghSpace.GetLength(0);
            int rhoMax = houghSpace.GetLength(1);
            for (int t = 0; t < thetaCount; t++)
            {
                for (int r = 0; r < rhoMax; r++)
                {
                    if (houghSpace[t, r] < peakThreshold)
                    {
                        houghSpace[t, r] = 0;
                    }
                }
            }

            // Step 3: Apply closing (dilation followed by erosion)
            byte[,] closedHoughSpace = closeImage(houghSpace, createStructuringElement("square", closingSize));

            // Step 4: Find peak centers
            List<Tuple<int, int>> peakCenters = FindPeakCenters(closedHoughSpace);

            // Step 5: Format peak centers into a human-readable string
            string formattedPeaks = "Peak Values:\n";
            foreach (var peak in peakCenters)
            {
                // Format each peak value with a new line after it
                formattedPeaks += $"(r: {peak.Item1}, θ: {peak.Item2})\n";
            }

            // Display the formatted peak values on your label
            peakValues.Text = formattedPeaks.TrimEnd('\n'); // Ensure no trailing newline

            return peakCenters;
        }


        private List<Tuple<int, int>> FindPeakCenters(byte[,] image)
        {
            List<Tuple<int, int>> peaks = new List<Tuple<int, int>>();
            int height = image.GetLength(0);
            int width = image.GetLength(1);

            // Implement a simple peak-finding algorithm (connected components or local maxima)
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    // Check if the current pixel is non-zero (potential peak)
                    if (image[y, x] > 0)
                    {
                        bool isPeak = true;

                        // Check neighboring pixels (8-neighbor connectivity)
                        for (int dy = -1; dy <= 1; dy++)
                        {
                            for (int dx = -1; dx <= 1; dx++)
                            {
                                int ny = y + dy;
                                int nx = x + dx;

                                // Make sure we're within bounds and not comparing the same point
                                if (dy != 0 || dx != 0)
                                {
                                    if (ny >= 0 && ny < height && nx >= 0 && nx < width)
                                    {
                                        if (image[ny, nx] > image[y, x])
                                        {
                                            isPeak = false;
                                            break;
                                        }
                                    }
                                }
                            }

                            if (!isPeak)
                                break;
                        }

                        // If it's a local peak, add it
                        if (isPeak)
                        {
                            peaks.Add(Tuple.Create(y, x)); // Add the (r, θ) pair
                        }
                    }
                }
            }

            return peaks;
        }








        private List<Tuple<Point, Point>> houghLineDetection(byte[,] image, Tuple<int, int> rThetaPair, int minIntensityThreshold, int minLength, int maxGap)
        {
            int width = image.GetLength(1);
            int height = image.GetLength(0);

            // Extract (r, theta) from the input tuple
            int r = rThetaPair.Item1;
            int theta = rThetaPair.Item2;

            // Precompute cos and sin for the given theta
            double thetaRadians = theta * Math.PI / 180.0;
            double cosTheta = Math.Cos(thetaRadians);
            double sinTheta = Math.Sin(thetaRadians);

            // List to store detected line segments
            List<Tuple<Point, Point>> lineSegments = new List<Tuple<Point, Point>>();

            // Variables to track the current segment
            List<Point> currentSegment = new List<Point>();
            int gapCount = 0;

            // Iterate over the entire image to check each pixel
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    // Compute the expected r for the given x, y
                    double expectedR = x * cosTheta + y * sinTheta;

                    // Check if the pixel lies on the line (with a small tolerance)
                    if (Math.Abs(expectedR - r) < 1.0) // Smaller tolerance
                    {
                        // Check if the pixel is "on" (foreground or above intensity threshold)
                        if (image[y, x] >= minIntensityThreshold)
                        {
                            currentSegment.Add(new Point(x, y));
                            gapCount = 0; // Reset gap count
                        }
                        else
                        {
                            // Pixel is "off" (background), check gap tolerance
                            gapCount++;
                            if (gapCount > maxGap)
                            {
                                // If the gap exceeds maxGap, finalize the current segment
                                if (currentSegment.Count >= minLength)
                                {
                                    // Add segment as a tuple of start and end points
                                    lineSegments.Add(Tuple.Create(currentSegment.First(), currentSegment.Last()));
                                }
                                currentSegment.Clear(); // Reset the segment
                                gapCount = 0;
                            }
                        }
                    }
                }
            }

            // Finalize any remaining segment after the loop
            if (currentSegment.Count >= minLength)
            {
                lineSegments.Add(Tuple.Create(currentSegment.First(), currentSegment.Last()));
            }

            return lineSegments;
        }


        private void PrintDetectedLines(List<Tuple<Point, Point>> lineSegments)
        {
            // Initialize the string that will hold the formatted lines
            string formattedLines = "Detected Line Segments:\n";

            // Loop through the list of line segments
            foreach (var segment in lineSegments)
            {
                Point start = segment.Item1;
                Point end = segment.Item2;

                // Append each line segment with the format: (startX, startY) -> (endX, endY)
                formattedLines += $"({start.X}, {start.Y}) -> ({end.X}, {end.Y})\n";
            }

            // Assign the formatted lines to a label or text element (similar to distinctValueLabel)
            lineSegmentsLabel.Text = formattedLines.TrimEnd('\n'); // Ensure no trailing newline
        }




        private byte[,] visualizeHoughLineSegments(byte[,] inputImage, List<Tuple<Point, Point>> lineSegments)
        {
            // Create a copy of the input image to avoid modifying the original
            int height = inputImage.GetLength(0);
            int width = inputImage.GetLength(1);
            byte[,] outputImage = new byte[height, width];

            // Copy the input image into the output image
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    outputImage[y, x] = inputImage[y, x];
                }
            }

            // Superimpose the detected line segments on the image
            foreach (var segment in lineSegments)
            {
                Point start = segment.Item1;
                Point end = segment.Item2;

                // Draw the line segment on the output image
                DrawLineSegment(outputImage, start, end);
            }

            return outputImage; // Return the image with superimposed line segments
        }

        // Helper function to draw a line segment between two points on the image
        private void DrawLineSegment(byte[,] image, Point start, Point end)
        {
            // Bresenham's line algorithm or any simple line drawing algorithm can be used here
            int x0 = start.X, y0 = start.Y;
            int x1 = end.X, y1 = end.Y;

            int dx = Math.Abs(x1 - x0);
            int dy = Math.Abs(y1 - y0);

            int sx = (x0 < x1) ? 1 : -1;
            int sy = (y0 < y1) ? 1 : -1;


            int err = dx - dy;

            while (true)
            {
                // Make sure the point is within bounds before drawing
                if (x0 >= 0 && x0 < image.GetLength(1) && y0 >= 0 && y0 < image.GetLength(0))
                {
                    // Set the pixel value to the maximum (255) to make the line visible
                    image[y0, x0] = 255;  // For binary or grayscale, use white for the line
                }

                // Break when we reach the end of the segment
                if (x0 == x1 && y0 == y1) break;

                int e2 = 2 * err;
                if (e2 > -dy)
                {
                    err -= dy;
                    x0 += sx;
                }
                if (e2 < dx)
                {
                    err += dx;
                    y0 += sy;
                }
            }
        }



        private byte[,] houghTransformAngleLimits(byte[,] inputData, int thetaAxisSize, int rAxisSize, bool isBinary, double lowerAngleLimit, double upperAngleLimit)
        {
            int width = inputData.GetLength(1);
            int height = inputData.GetLength(0);
            int maxRadius = (int)Math.Ceiling(Math.Sqrt(width * width + height * height));
            int halfRAxisSize = rAxisSize >> 1; // Bit shift to divide by 2
            int[,] accumulator = new int[thetaAxisSize, rAxisSize]; // Use int for the accumulator initially

            // Precompute sin and cos tables for theta values
            double[] sinTable = new double[thetaAxisSize];
            double[] cosTable = new double[thetaAxisSize];

            // Convert angle limits from degrees to radians
            double lowerLimitRadians = lowerAngleLimit * Math.PI / 180.0;
            double upperLimitRadians = upperAngleLimit * Math.PI / 180.0;

            // Compute the range of theta values based on limits (theta axis maps [0, thetaAxisSize) to [0, π))
            int lowerThetaIndex = (int)(lowerLimitRadians * thetaAxisSize / Math.PI) % thetaAxisSize;
            int upperThetaIndex = (int)(upperLimitRadians * thetaAxisSize / Math.PI) % thetaAxisSize;

            // Fill sin and cos tables
            for (int theta = 0; theta < thetaAxisSize; theta++)
            {
                double thetaRadians = theta * Math.PI / thetaAxisSize;
                sinTable[theta] = Math.Sin(thetaRadians);
                cosTable[theta] = Math.Cos(thetaRadians);
            }

            // Apply Hough Transform on the entire image
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    byte pixelValue = inputData[y, x];

                    if (pixelValue > 0) // Only process non-zero pixels
                    {
                        for (int theta = lowerThetaIndex; theta <= upperThetaIndex; theta++)
                        {
                            // Center the coordinates
                            double r = cosTable[theta] * x + sinTable[theta] * y;
                            int rScaled = (int)Math.Round(r * halfRAxisSize / maxRadius) + halfRAxisSize;

                            if (rScaled >= 0 && rScaled < rAxisSize) // Ensure it's within bounds
                            {
                                // Increment the accumulator differently based on whether it's binary or grayscale
                                if (isBinary)
                                {
                                    accumulator[theta, rScaled]++; // For binary, increment by 1
                                }
                                else
                                {
                                    accumulator[theta, rScaled] += pixelValue; // For grayscale, increment by pixel value
                                }
                            }
                        }
                    }
                }
            }

            // Normalize accumulator to byte[,] for output
            int maxAccValue = accumulator.Cast<int>().Max();
            byte[,] outputData = new byte[thetaAxisSize, rAxisSize];

            if (maxAccValue == 0) return outputData; // nothing to normalize

            for (int theta = 0; theta < thetaAxisSize; theta++)
            {
                for (int r = 0; r < rAxisSize; r++)
                {
                    outputData[theta, r] = (byte)Math.Min(accumulator[theta, r] * 255 / maxAccValue, 255); // Scale to byte range
                }
            }

            return outputData;
        }



        // Assignment 4: Object Detection

        private byte[,] preProcessing(byte[,] inputImage)
        {

            // Step 1: Apply Gaussian filter to reduce noise and smooth the image
            float[,] gaussianFilter = createGaussianFilter(5, 3f);
            byte[,] smoothedImage = convolveImage(inputImage, gaussianFilter);

            // Step 2: Reduce noise further by morphological opening
            byte structureElementSize = 3;
            byte[,] structureElement = createStructuringElement("square", structureElementSize);
            byte[,] openedImage = openImage(smoothedImage, structureElement);

            // Step 3: Detect edges after noise reduction for cleaner edges
            byte[,] edgeImage = edgeMagnitude(openedImage, horizontalKernel, verticalKernel);

            // Step 4: Sharpen the edges to enhance edge contrast
            byte[,] sharpenedImage = edgeSharpening(edgeImage);

            // Step 5: Increase the contrast of the final image to boost object features
            byte[,] finalImage = adjustContrast(sharpenedImage);

            return finalImage;
        }



        private byte[,] DetectStopSign(byte[,] edgeImage, int thetaAxisSize, int rAxisSize, int peakThreshold, int closingSize)
        {
            // Step 1 & 2: Peak finding (PeakFinding internally applies the Hough Transform)
            List<Tuple<int, int>> linePeaks = PeakFinding(edgeImage, peakThreshold, closingSize);

            // Step 3: Identify Octagonal Shape
            List<Tuple<int, int>> octagonPeaks = FindOctagonPeaks(linePeaks, edgeImage.GetLength(0), edgeImage.GetLength(1));

            // Step 4: Initialize the output image
            byte[,] outputImage = new byte[edgeImage.GetLength(0), edgeImage.GetLength(1)];

            // Copy the edge image to the output image for visibility
            Array.Copy(edgeImage, outputImage, edgeImage.Length);

            double distanceThreshold = 20.0; // Adjustable threshold for detecting central peaks around centroid
            int padding = 20; // Additional padding to expand bounding box

            // List to hold the octagon points for the bounding box
            List<Tuple<int, int>> objectBoundingBox = new List<Tuple<int, int>>();

            if (octagonPeaks.Count == 8) // Valid octagon detected
            {
                // Calculate centroid of the octagon
                var centroid = CalculateCentroid(octagonPeaks);

                // Check for central peaks near the centroid
                var centralPeaks = GetCentralPeaks(linePeaks, centroid, distanceThreshold);

                if (centralPeaks.Count > 0)
                {
                    // Step 5: Convert Hough peaks to (x, y) coordinates in the original image
                    List<Tuple<int, int>> octagonPoints = new List<Tuple<int, int>>();
                    foreach (var peak in octagonPeaks)
                    {
                        int r = peak.Item1;
                        int theta = peak.Item2;

                        // Convert (r, theta) to (x, y) coordinates
                        double thetaRad = theta * Math.PI / 180; // Convert theta to radians
                        int x = (int)(Math.Cos(thetaRad) * r);
                        int y = (int)(Math.Sin(thetaRad) * r);

                        // Ensure (x, y) is within image bounds
                        x = Math.Max(0, Math.Min(x, edgeImage.GetLength(1) - 1));
                        y = Math.Max(0, Math.Min(y, edgeImage.GetLength(0) - 1));

                        octagonPoints.Add(new Tuple<int, int>(x, y));
                        objectBoundingBox.Add(new Tuple<int, int>(x, y));
                    }

                    // Step 6: Calculate the bounding box in the original image with padding
                    int minX = Math.Max(0, octagonPoints.Min(p => p.Item1) - padding);
                    int maxX = Math.Min(edgeImage.GetLength(1) - 1, octagonPoints.Max(p => p.Item1) + padding);
                    int minY = Math.Max(0, octagonPoints.Min(p => p.Item2) - padding);
                    int maxY = Math.Min(edgeImage.GetLength(0) - 1, octagonPoints.Max(p => p.Item2) + padding);

                    // Additional check to ensure bounding box covers all relevant area
                    if (objectBoundingBox.Count > 0)
                    {
                        minX = Math.Min(minX, objectBoundingBox.Min(p => p.Item1));
                        maxX = Math.Max(maxX, objectBoundingBox.Max(p => p.Item1));
                        minY = Math.Min(minY, objectBoundingBox.Min(p => p.Item2));
                        maxY = Math.Max(maxY, objectBoundingBox.Max(p => p.Item2));
                    }

                    // Step 7: Draw the bounding box on the output image
                    // Use 128 to represent red color
                    byte redColorValue = 128;

                    // Draw top and bottom lines of the bounding box
                    for (int x = minX; x <= maxX; x++)
                    {
                        outputImage[minY, x] = redColorValue; // Red for top edge
                        outputImage[maxY, x] = redColorValue; // Red for bottom edge
                    }
                    // Draw left and right lines of the bounding box
                    for (int y = minY; y <= maxY; y++)
                    {
                        outputImage[y, minX] = redColorValue; // Red for left edge
                        outputImage[y, maxX] = redColorValue; // Red for right edge
                    }
                }
            }

            return outputImage; // Return the output image with the bounding box around the detected stop sign
        }







        private List<Tuple<int, int>> FindOctagonPeaks(List<Tuple<int, int>> peaks, int height, int width)
        {
            List<Tuple<int, int>> octagonPeaks = new List<Tuple<int, int>>();
            double angleTolerance = 20; // Allowable tolerance for angles in degrees

            // Step 1: Filter peaks to form groups with 8 edges
            for (int i = 0; i < peaks.Count; i++)
            {
                List<Tuple<int, int>> potentialOctagon = new List<Tuple<int, int>>();
                potentialOctagon.Add(peaks[i]);

                // Look for additional peaks that can form an octagon
                for (int j = i + 1; j < peaks.Count && potentialOctagon.Count < 8; j++)
                {
                    var currentPeak = peaks[j];
                    var lastAddedPeak = potentialOctagon[potentialOctagon.Count - 1];

                    // Check angle between the two peaks
                    double angle = CalculateAngleBetweenPeaks(lastAddedPeak, currentPeak, width, height);

                    // If the angle is approximately 135 degrees, consider adding it to the potential octagon
                    if (Math.Abs(angle - 135.0) <= angleTolerance)
                    {
                        potentialOctagon.Add(currentPeak);
                    }
                }

                // Check if we have a valid octagon (8 peaks connected by valid angles)
                if (potentialOctagon.Count == 8)
                {
                    octagonPeaks = potentialOctagon;
                    break; // Stop once we've found an octagon
                }
            }

            return octagonPeaks;
        }

        private Tuple<double, double> CalculateCentroid(List<Tuple<int, int>> octagonPeaks)
        {
            double sumX = 0;
            double sumY = 0;

            foreach (var peak in octagonPeaks)
            {
                sumX += peak.Item1; // r
                sumY += peak.Item2; // theta
            }

            int count = octagonPeaks.Count;
            return new Tuple<double, double>(sumX / count, sumY / count);
        }

        private List<Tuple<int, int>> GetCentralPeaks(List<Tuple<int, int>> peaks, Tuple<double, double> centroid, double distanceThreshold)
        {
            List<Tuple<int, int>> centralPeaks = new List<Tuple<int, int>>();

            foreach (var peak in peaks)
            {
                double distance = Math.Sqrt(Math.Pow(centroid.Item1 - peak.Item1, 2) + Math.Pow(centroid.Item2 - peak.Item2, 2));
                if (distance <= distanceThreshold) // Check if the peak is within a certain distance from the centroid
                {
                    centralPeaks.Add(peak);
                }
            }

            return centralPeaks;
        }

        private double CalculateAngleBetweenPeaks(Tuple<int, int> peak1, Tuple<int, int> peak2, int imageWidth, int imageHeight)
        {
            // Convert peak (r, theta) to (x, y) coordinates
            double r1 = peak1.Item1;
            double theta1 = peak1.Item2 * Math.PI / 180; // Convert theta to radians

            double r2 = peak2.Item1;
            double theta2 = peak2.Item2 * Math.PI / 180; // Convert theta to radians

            // Calculate the angle between two peaks based on their theta values
            double deltaX = Math.Cos(theta2) - Math.Cos(theta1);
            double deltaY = Math.Sin(theta2) - Math.Sin(theta1);

            // Return the angle in degrees
            return Math.Atan2(deltaY, deltaX) * (180.0 / Math.PI); // Convert to degrees
        }

    }
}
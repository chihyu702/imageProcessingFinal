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
            PipelineOne,
            PipelineTwo,
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
            imageX,
            imageY,
            En,
            Fn,
            Gn_and_Hn,

            visualizeHoughLineSegments
        }

        /*
         * these are the parameters for your processing functions, you should add more as you see fit
         * it is useful to set them based on controls such as sliders, which you can add to the form
         */
        private byte filterSize = 5;
        private float filterSigma = 1f;
        private byte threshold = 127;
        private byte structureElementSize = 7;

        public INFOIBV()
        {
            InitializeComponent();
            populateCombobox();
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



        /*
 * applyButton_Click: process when user clicks "Apply" button
 */
        private void applyButton_Click(object sender, EventArgs e)
        {
            if (InputImage == null)
            {
                MessageBox.Show("No input image loaded.");
                return; // exit if no input image
            }

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

            byte[,] workingImage = convertToGrayscale(Image); // Convert first image to grayscale

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
            else
            {
                // give warning that two images are required for AND/OR operations
                MessageBox.Show("Two images are required for the AND/OR operations");
            }

            // Apply processing with or without the second image
            workingImage = applyProcessingFunction(workingImage, secondImage); // Pass null if secondImage is not loaded

            // Copy array to output Bitmap
            for (int x = 0; x < workingImage.GetLength(0); x++)
            {
                for (int y = 0; y < workingImage.GetLength(1); y++)
                {
                    Color newColor = Color.FromArgb(workingImage[x, y], workingImage[x, y], workingImage[x, y]);
                    OutputImage.SetPixel(x, y, newColor); // set the pixel color at coordinate (x, y)
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
                    // define these kernels yourself
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
                    return edgeMagnitude(workingImage, horizontalKernel, verticalKernel);
                case ProcessingFunctions.Threshold:
                    return thresholdImage(workingImage, threshold);
                case ProcessingFunctions.EdgeSharpening:
                    return edgeSharpening(workingImage);
                case ProcessingFunctions.HistogramEqualization:
                    return histogramEqualization(workingImage);
                case ProcessingFunctions.PipelineOne:
                    return pipelineOne(workingImage);
                case ProcessingFunctions.PipelineTwo:
                    return pipelineTwo(workingImage);
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
                        // testing
                        workingImage = thresholdImage(workingImage, threshold);
                        secondImage = thresholdImage(secondImage, threshold);
                        return andImages(workingImage, secondImage); // use secondImage for andImages
                    }
                    else
                    {
                        MessageBox.Show("Second image is required for AND operation. Please load another image.");
                        return workingImage;  // Return the original image if secondImage is missing
                    }
                case ProcessingFunctions.orImages:
                    if (secondImage != null)
                    {
                        workingImage = thresholdImage(workingImage, threshold);
                        secondImage = thresholdImage(secondImage, threshold);
                        return orImages(workingImage, secondImage); // use secondImage for orImages
                    }
                    else
                    {
                        MessageBox.Show("Second image is required for OR operation. Please load another image.");
                        return workingImage;  // Return the original image if secondImage is missing
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
                case ProcessingFunctions.imageX:
                    // dilate image W with a 3x3 square kernel.This is image X.
                    // image W: make sure image B from assignment 1 is a binary single channel image
                    return dilateImage(pipelineOne(workingImage), createStructuringElement("square", structureElementSize), true);
                case ProcessingFunctions.imageY:
                    // erode image W with a 3x3 square kernel. This is image Y.
                    return erodeImage(pipelineOne(workingImage), createStructuringElement("square", structureElementSize), true);

                
                case ProcessingFunctions.complementImage:
                    return complementImage(pipelineOne(workingImage));

                case ProcessingFunctions.En:
                    return dilateImage(workingImage, createStructuringElement("square", 11), false);

                case ProcessingFunctions.Fn:
                    distinctValueLabel.Visible = true;
                    return valueCounting(dilateImage(workingImage, createStructuringElement("square", 109), false));


                case ProcessingFunctions.Gn_and_Hn:
                    distinctValueLabel.Visible = true;
                    return Gn_and_Hn_Func(workingImage);

                case ProcessingFunctions.VisualizeHoughLineSegments:
                    return visualizeHoughLineSegments(workingImage)


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
            progressBar.Maximum = InputImage.Size.Width * InputImage.Size.Height;
            progressBar.Value = 1;
            progressBar.Step = 1;

            // process all pixels in the image
            for (int x = 0; x < InputImage.Size.Width; x++)                 // loop over columns
                for (int y = 0; y < InputImage.Size.Height; y++)            // loop over rows
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
                for (int y=0; y < inputImage.GetLength(1); y++)
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

            // TODO: add your functionality and checks
            byte a_low = 0;
            byte a_high = 255;

            // find the current min and max intensity values
            for (int x = 0; x < inputImage.GetLength(0); x++)
            {
                for (int y = 0; y < inputImage.GetLength(1); y++)
                {
                    if (inputImage[x, y] < a_low)
                    {
                        a_low = inputImage[x, y];
                    }
                    else if (inputImage[x, y] > a_high)
                    {
                        a_high = inputImage[x, y];
                    }
                }
            }

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
                    for (int i = 0; i < filterSize; i++)
                    {
                        for (int j = 0; j < filterSize; j++)
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

        private byte[,] Gn_and_Hn_Func(byte[,] inputImage)
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


        /*
         * HoughLineDetection: 
         * input:   inputImage          single-channel (byte) binary image
         *          (r, theta)          pair from peakFinding function
         *          minIntThreshold     minimum intensity threshold (for grayscale edge strength images)
         *          minLength           minimum length 
         *          maxGap              maximum gap
         * output:   list               list of line segments
         */
        private List<LineSegment> HoughLineDetection(byte[,] image, List<<double, double>> peakFinding, double minIntensityThreshold, int minLength, int maxGap)
        {
            int width = image.GetLength(1);
            int height = image.GetLength(0);
            // empty to store the list of segment
            List<LineSegment> segments = new List<LineSegment>();

            // Get each (r, theta) pair
            foreach (var rTheta in peakFinding)
            {
                double r = peakFinding.Item1;
                double theta = peakFinding.Item2;

                // compute cos and sin of the theta
                double cosTheta = Math.Cos(theta);
                double sinTheta = Math.Sin(theta);

                // Traverse over the image and find points along the line
                List<Point> currentSegment = new List<Point>();
                int gapCounter = 0;

                for (int x = 0; x < width; x++)
                {
                    // get y value for the given r and theta
                    int y = (int)((r - x * cosTheta) / sinTheta);

                    if (y >= 0 && y < height)
                    {
                        // Check if the adjacent pixels are "on" 
                        // foreground in case of a binary image
                        // and above the minimum intensity threshold for grayscale images
                        if (image[y, x] >= minIntensityThreshold)
                        {
                            currentSegment.Add(new Point(x, y));
                            gapCounter = 0;  
                        }
                        else
                        {
                            gapCounter++;
                            if (gapCounter > maxGap)
                            {
                                if (currentSegment.Count >= minLength)
                                {
                                    // Store the segment if it's long enough
                                    segments.Add(new LineSegment
                                    {
                                        Start = currentSegment[0],
                                        End = currentSegment[currentSegment.Count - 1]
                                    });
                                }
                                // Reset current segment
                                currentSegment.Clear();
                                gapCounter = 0;
                            }
                        }
                    }
                }

                // If the current segment is longer than the minimum length parameter, add to the list
                if (currentSegment.Count >= minLength)
                {
                    segments.Add(new LineSegment
                    {
                        Start = currentSegment[0],
                        End = currentSegment[currentSegment.Count - 1]
                    });
                }
            }

            return segments;
        }


        /*
         * visualizeHoughLineSegments: 
         * input:   inputImage          single-channel (byte) image
         *          HoughLineDetector   list of line segment descriptions
         * output:                      image with the corresponding line segments superimposed
         */
        private byte[,] visualizeHoughLineSegments(byte[,] inputImage, List<LineSegment> HoughLineDetection)
        {
            // Create a temporary image dark background
            byte[,] tempImage = new byte[,](inputImage.Width, inputImage.Height);


            using (Graphics g = Graphics.FromImage(tempImage))
            {
                // Draw the detected line segments in white (bright lines)
                using (Pen linePen = new Pen(Color.White, 2))
                {
                    foreach (var segment in HoughLineDetection)
                    {
                        g.DrawLine(linePen, segment.Start, segment.End);
                    }
                }
            }

            return tempImage;
        }
}
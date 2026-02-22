# ImageProcessFinal

This repo is for image processing courseworks at Utrecht University (FA24)  
A Windows Forms application built with C# (.NET Framework 4.8) for applying various image processing algorithms.



## Features


- **Basic Operations**: Invert, Adjust Contrast, Threshold, Histogram Equalization
- **Filtering**: Gaussian (Convolution) Filter, Median Filter, Edge Sharpening
- **Edge Detection**: Sobel edge magnitude
- **Morphological Operations**: Dilate, Erode, Open, Close, Complement
- **Shape Analysis**: Trace Boundary, Largest Shape, Connected Components (Count Non-Background Values), Value Counting
- **Hough Transform**: Standard, Angle-Limited, Peak Finding, Line Detection, Line Segment Visualization, Circle Detection
- **Multi-image Operations**: AND, OR (requires two loaded images)
- **Stop Sign Detection**: Full pipeline combining preprocessing, Hough transform, and peak finding


## Requirements


- Windows OS
- .NET Framework 4.8
- Visual Studio (to build from source)


## Getting Started


1. Clone the repository:
  ```
https://github.com/chihyu702/imageProcessingFinal.git
  ```
2. Open `INFOIBV.sln` in Visual Studio.
3. Build and run the project (x86, Debug or Release).


## Usage


1. Click **Load** to load an input image (max 512x512).
2. (Optional) Load a second image for AND/OR operations.
3. Select a processing function from the dropdown.
4. Fill in any required parameters (peak threshold, filter size, angle limits, etc.).
5. Click **Apply** to process the image.
6. Click **Save** to save the output image.

## Usage example
<img width="1707" height="652" alt="image" src="https://github.com/user-attachments/assets/cfa50bf6-ea07-4bf2-84b9-d915f2b45fbd" />



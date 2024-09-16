# Geometric Shape Detection and Color Identification

## 1. Shape Detection
This part of the project involves detecting and classifying four geometric shapes in an image:
- **Rectangle**
- **Square**
- **Circle**
- **Triangle**

### Methodology:
- **Edge Detection**: We utilize Canny Edge Detection to capture the shape boundaries from the image.
- **Contour Analysis**: The contours of the shapes are extracted using OpenCV's `cv2.findContours` function.
- **Shape Classification**: The shapes are classified based on the number of vertices in the approximated contour using `cv2.approxPolyDP`. 
  - Shapes are identified as:
    - **Triangle**: If the contour has 3 vertices.
    - **Rectangle** or **Square**: If the contour has 4 vertices, where the aspect ratio determines if it is a square or rectangle.
    - **Circle**: If the contour has more than 4 vertices.

## 2. Color Identification
Each detected shape is then analyzed to identify its color (red, blue, green, or yellow).

### Methodology:
- **Color Space Conversion**: Convert the image from BGR to HSV color space for more accurate color detection using OpenCV's `cv2.cvtColor`.
- **Color Thresholding**: Predefined HSV color ranges are used to detect the color of each shape. The HSV values for each color are set to detect:
  - **Red**
  - **Blue**
  - **Green**
  - **Yellow**
- **Color Assignment**: By sampling the middle point of each shape, the HSV values are checked to determine the corresponding color.

## 3. Implementation
The entire solution is implemented using **Python** and **OpenCV**, focusing on classical image processing methods to identify both shape and color. The goal is to detect shapes and colors in images that consist of simple, animated shapes, not real-world photographs.

## 4. Challenges and Insights
This task was designed to explore classical image processing techniques without the use of deep learning models. Some challenges faced during development include:
- Ensuring accurate shape detection using contour analysis.
- Fine-tuning HSV values for reliable color detection across different lighting conditions.

## 5. Conclusion
By combining classical edge detection, contour analysis, and color thresholding techniques, the program successfully identifies and classifies the shapes and their colors. This project highlights the power of classical methods in computer vision for solving simple geometric problems.

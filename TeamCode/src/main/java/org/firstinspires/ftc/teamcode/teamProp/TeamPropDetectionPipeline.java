/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.teamProp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

class TeamPropDetectionPipeline extends OpenCvPipeline
{
    private Mat currentImage = new Mat();
    // Matrices for OpenCv
    //Mat colorImage = new Mat();
    private Mat blurImage = new Mat();
    private Mat hsvImage = new Mat();
    private Mat maskHSVBlue = new Mat();
    private Mat maskHSVRed1 = new Mat();
    private Mat maskHSVRed2 = new Mat();
    private Mat maskedImage = new Mat();
    private Mat maskHSVRed = new Mat();
    private Mat grey = new Mat();
    private final Object detectionsUpdateSync = new Object();

    Mat cameraMatrix;

    Scalar blue = new Scalar(7,197,235);
    Scalar red = new Scalar(255,0,0);
    Scalar green = new Scalar(0,255,0);
    Scalar white = new Scalar(255,255,255);

    double fx;
    double fy;
    double cx;
    double cy;

    private Point position = new Point(0,0);

    Telemetry telemetry;
    //Color specific filtering values
    static final double minBlueInitValues[]= {90, 90, 20};
    static final double maxBlueInitValues[] = {135, 255, 255};

    //It is on 2 different sections in the hsv therefore we need to merge the 2 red sections
    static final double minRedInitValues1[]= {165, 125, 110};
    static final double maxRedInitValues1[] = {255, 255, 255};
    static final double minRedInitValues2[]= {0, 125, 110};
    static final double maxRedInitValues2[] = {15, 255, 255};

    // Blacken out the top of the image, because the team prop can only be in the bottom half
    static final int Y_BLACK_COORDINATE = 290;

    public TeamPropDetectionPipeline(double fx, double fy, double cx, double cy, Telemetry telemetry)
    {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        this.telemetry = telemetry;
        constructMatrix();

    }

    @Override
    public Mat processFrame(Mat colorImage) {
        // Initial blur to filter out some unwanted points
        Imgproc.GaussianBlur(colorImage, blurImage, new Size(9.0, 9.0), 75, 75);

        // Convert image in HSV to filter easily on hue for blue and red filtering
        Imgproc.cvtColor(blurImage, hsvImage, Imgproc.COLOR_RGB2HSV);


        // blue HSV filter
        Core.inRange(hsvImage,new Scalar(minBlueInitValues),new Scalar(maxBlueInitValues), maskHSVBlue);

        // red HSV filter
        Core.inRange(hsvImage,new Scalar(minRedInitValues1),new Scalar(maxRedInitValues1), maskHSVRed1);
        Core.inRange(hsvImage,new Scalar(minRedInitValues2),new Scalar(maxRedInitValues2), maskHSVRed2);

        // Merge both red hsv filters
        Core.max(maskHSVRed1, maskHSVRed2, maskHSVRed);

        // Define a rectangle that is the top of the image (it relates to the image)
        Mat subImg1 = maskHSVRed.submat(new Rect(0,0, maskHSVRed.cols(), Y_BLACK_COORDINATE));
        Mat subImg2 = maskHSVBlue.submat(new Rect(0,0, maskHSVBlue.cols(), Y_BLACK_COORDINATE));
        Mat subImg3 = colorImage.submat(new Rect(0,0, maskHSVBlue.cols(), Y_BLACK_COORDINATE));
        // Set this rectangle to black
        subImg1.setTo(new Scalar (0,0,0));
        subImg2.setTo(new Scalar (0,0,0));
        subImg3.setTo(new Scalar (0,0,0));


        // Decide which mask to treat based on amount of red or blue in the image
        // (if we are blue alliance, there will be more blue due to the proximity of the team prop and vice versa)
        int numBluePixels = Core.countNonZero(maskHSVBlue);
        int numRedPixels = Core.countNonZero(maskHSVRed);

        if (numBluePixels > numRedPixels){
            currentImage = maskHSVBlue;
        }
        else if(numRedPixels > numBluePixels){
            currentImage = maskHSVRed;
        }
        else {
            currentImage = maskHSVBlue;
        }


        // Finding contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(currentImage, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        int numContours = contours.size();

        // No rectangle detected
        if(numContours==0) {
            setPosition (0, 0);
            return colorImage;
        }
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[numContours];
        RotatedRect[] rectangle = new RotatedRect[numContours];
        double maxArea = 0;
        int currentBiggestRectangle = -1;
        // Going through all the contours to find the rectangles
        for (int i = 0; i < numContours; i++) {


            // Getting only the rectangles from the contours
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            rectangle[i] = Imgproc.minAreaRect(contoursPoly[i]);

            // Filtering out rectangles with too small of an area
            if(rectangle[i].size.area() <= 100) {
                continue;
            }

            // Filtering out rectangles that look too much like lines
            double rectangleRatio = Math.max(rectangle[i].size.width,rectangle[i].size.height)/Math.min(rectangle[i].size.width,rectangle[i].size.height);


            if (rectangleRatio > 4){
                Point[] vertices = new Point[4];
                rectangle[i].points(vertices);
                List<MatOfPoint> box = new ArrayList<>();
                box.add(new MatOfPoint(vertices));

                Imgproc.drawContours(colorImage, box, -1, red, 3, 0);
                continue;
            }

            // Taking only the biggest rectangle area
            double currentArea = rectangle[i].size.area();
            if (currentArea > maxArea){
                maxArea = currentArea;
                currentBiggestRectangle = i;
            }

            Point[] vertices = new Point[4];
            rectangle[currentBiggestRectangle].points(vertices);
            List<MatOfPoint> box = new ArrayList<>();
            box.add(new MatOfPoint(vertices));

            Imgproc.drawContours(colorImage, box, -1, green, 3, 0);
        }

        // Drawing contours is possible for for debugging
        telemetry.addLine("# of contours" + numContours);

        if (currentBiggestRectangle != -1) {

            Point[] vertices = new Point[4];
            rectangle[currentBiggestRectangle].points(vertices);
            List<MatOfPoint> box = new ArrayList<>();
            box.add(new MatOfPoint(vertices));

            Imgproc.drawContours(colorImage, box, -1, blue, 3, 0);

            setPosition(rectangle[currentBiggestRectangle].center.x, rectangle[currentBiggestRectangle].center.y);
        }

        return colorImage;
    }


    public void setPosition(double x, double y)
    {
        synchronized (detectionsUpdateSync)
        {
            position = new Point (x, y);
        }
    }

    public Point getLatestPosition()
    {
        synchronized (detectionsUpdateSync)
        {
            return position;
        }
    }


    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }


    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    class Pose
    {
        Mat rvec;
        Mat tvec;

        public Pose()
        {
            rvec = new Mat(3, 1, CvType.CV_32F);
            tvec = new Mat(3, 1, CvType.CV_32F);
        }

        public Pose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }
}
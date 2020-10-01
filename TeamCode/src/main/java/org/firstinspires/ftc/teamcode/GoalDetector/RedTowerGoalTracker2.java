/*
 * Copyright (c) 2020 OpenFTC Team
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

package org.firstinspires.ftc.teamcode.GoalDetector;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="GoalDetector", group ="Test")
public class RedTowerGoalTracker2 extends LinearOpMode
{
    OpenCvInternalCamera camera;

    Pipeline pipeline = new Pipeline();

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.setPipeline(pipeline);
                camera.startStreaming(1280,720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        while (opModeIsActive())
        {
            telemetry.addLine(String.format("X: %.2fft", pipeline.translation[0]/254));
            telemetry.addLine(String.format("Y: %.2fft", pipeline.translation[1]/254));
            telemetry.addLine(String.format("Z: %.2fft", pipeline.translation[2]/254));
            telemetry.update();
            sleep(20);
        }
    }

    class Pipeline extends OpenCvPipeline
    {
        Mat ycrcb = new Mat();
        Mat cr = new Mat();
        Mat cb = new Mat();
        Mat cr_normalized = new Mat();

        Mat cr_thresholded = new Mat();
        Mat cb_thresholded = new Mat();
        Mat combinedThesh = new Mat();

        Mat grey = new Mat();

        volatile int step = 0;
        final int maxstep = 5;

        Mat cameraMatrix;

        boolean shouldBreak = false;

        Scalar blue = new Scalar(7,197,235);
        Scalar red = new Scalar(219,0,1920);
        Scalar green = new Scalar(0,255,0);

        double fx = 1119.3;
        double fy = 1111.662;
        double cx = 638.553;
        double cy = 356.238;

        // UNITS ARE MM
        //double tagsizeX = 112;
        //double tagsizeY = 86;
        double tagsizeX = 11*25.4;
        double tagsizeY = 8.5*25.4;

        volatile double[] translation = new double[3];

        public Pipeline()
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

        void doColorConversion(Mat input)
        {
            Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(ycrcb, cr, 1);
            Core.extractChannel(ycrcb, cb, 2);
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
        }

        @Override
        public Mat processFrame(Mat input)
        {
            doColorConversion(input);

            //Core.normalize(cr, cr_normalized, 0, 255, Core.NORM_MINMAX);

            //Imgproc.threshold(cr_normalized, cr_thresholded, 190, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(cr, cr_thresholded, 190, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(cb, cb_thresholded, 128, 255, Imgproc.THRESH_BINARY_INV);

            Core.bitwise_and(cr_thresholded, cb_thresholded, combinedThesh);

            ArrayList<MatOfPoint> contours = new ArrayList<>();
            MatOfInt tree = new MatOfInt();

            Imgproc.findContours(combinedThesh, contours, tree, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            ContourTree3 contourTree = new ContourTree3(tree, contours);
            ArrayList<Contour> parents = new ArrayList<>(contourTree.rootContours);

            ContourTree3.filterBySqrtArea(60, parents);

            for(Contour c : parents)
            {
                ContourTree3.filterByArea(Math.pow(30, 2), c.children);

                if(c.children.size() == 1 && c.children.get(0).children.size() == 0)
                {
                    MatOfInt convex_hull = new MatOfInt();
                    Imgproc.convexHull(new MatOfPoint(c.children.get(0).self), convex_hull);

                    Point[] trapezoidCorners = sortPointsCounterclockwiseStartingBottomLeft(
                            calculateSubpixelCorners(
                                    cr,
                                    doTrapezoidFit(convex_hull, c.children.get(0).self),
                                    10));

                    Pose pose = poseFromTrapezoid(trapezoidCorners, cameraMatrix, tagsizeX, tagsizeY);

                    int dotSize = getCornerDotSize(pose);
                    Imgproc.circle(input, trapezoidCorners[0], dotSize, new Scalar(255,0,0), -1);
                    Imgproc.circle(input, trapezoidCorners[1], dotSize, new Scalar(0,255,0), -1);
                    Imgproc.circle(input, trapezoidCorners[2], dotSize, new Scalar(0,0,255), -1);
                    Imgproc.circle(input, trapezoidCorners[3], dotSize, new Scalar(0,0,0), -1);

                    translation[0] = pose.tvec.get(0,0)[0];
                    translation[1] = pose.tvec.get(1,0)[0];
                    translation[2] = pose.tvec.get(2,0)[0];

                    drawAxisMarker(input, tagsizeY/2.0, getAxisSize(pose), pose.rvec, pose.tvec, cameraMatrix);
                    //draw3dCubeMarker(input, 40, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
                }
            }

            switch (step)
            {
                case 0:
                    return input;

                case 1:
                    return cr;

                case 2:
                    return cb;

                case 3:
                    return cr_thresholded;

                case 4:
                    return cb_thresholded;

                case 5:
                    return combinedThesh;
            }

            return input;
        }

        public int getAxisSize(Pose pose)
        {
            // At 1ft (254mm) we make them size 10

            double size = (1/pose.tvec.get(2,0)[0])*254*20;

            System.out.println(size);

            return (int) Math.round(Math.max(size, 1.0));
        }

        public int getCornerDotSize(Pose pose)
        {
            // At 1ft (254mm) we make them size 20

            double size = (1/pose.tvec.get(2,0)[0])*254*20;

            System.out.println(size);

            return (int) Math.round(Math.max(size, 1.0));
        }

        @Override
        public void onViewportTapped()
        {
            camera.setExposureLocked(true);
            shouldBreak = true;

            int tempStep = step+1;
            if(tempStep > maxstep)
            {
                step = 0;
            }
            else
            {
                step = tempStep;
            }
        }

        /**
         * Takes an array of 4 points which define a quadrilateral,
         * and sorts them into counterclockwise order, starting from
         * the bottom left
         *
         * @param points the points to be sorted
         * @return the same points as passed in, except sorted into the
         *         order described.
         */
        Point[] sortPointsCounterclockwiseStartingBottomLeft(Point[] points)
        {
            //   We want to sort the points into the following order:
            //
            //   ------------------------------------------------
            //   | [3]                                      [2] |
            //   |                                              |
            //   |                                              |
            //   |                                              |
            //   |                                              |
            //   |                                              |
            //   |                                              |
            //   |                                              |
            //   |                                              |
            //   | [0]                                      [1] |
            //   ------------------------------------------------
            //

            // An array to hold the points in the newly sorted order
            Point[] pointsSorted = new Point[4];

            //------------------------------------------------
            // First we handle the left side points
            //------------------------------------------------

            // We go through all the points for a 1st time, looking for the
            // point which has the smallest X value.
            Point smallest_x_seen = null;
            for (int i = 0; i < 4; i++)
            {
                if (smallest_x_seen == null || points[i].x < smallest_x_seen.x)
                {
                    smallest_x_seen = points[i];
                }
            }

            // Now we go through again, again looking for the point which has the
            // smallest X value, except we skip over the point we just found before.
            Point smallest_x2_seen = null;
            for (int i = 0; i < 4; i++)
            {
                if ((smallest_x2_seen == null || points[i].x < smallest_x2_seen.x) && points[i] != smallest_x_seen)
                {
                    smallest_x2_seen = points[i];
                }
            }

            // Ok we now know the two points which have the smallest X value.
            // To determine which indexes we need to put them at, we now turn our
            // attention to their Y values. We need to put the one with the larger
            // Y value at index 0, and the one with the smaller Y value at index 3
            // (see the graphic at the beginning of this method)
            if (smallest_x_seen.y > smallest_x2_seen.y)
            {
                pointsSorted[0] = smallest_x_seen;
                pointsSorted[3] = smallest_x2_seen;
            }
            else
            {
                pointsSorted[0] = smallest_x2_seen;
                pointsSorted[3] = smallest_x_seen;
            }

            //------------------------------------------------
            // Now we handle the right side points
            //------------------------------------------------

            // Since we only have 4 points, and we already identified previously
            // the 2 with the smallest X value, we implicitly know which ones will
            // have the largest X values. So we can just go through the list of points
            // and find the two we *didn't* choose up above
            int num = 1;
            Point largest_x1 = null;
            Point largest_x2 = null;
            for(int i = 0; i < 4; i++)
            {
                if(points[i] != pointsSorted[0] && points[i] != pointsSorted[3])
                {
                    if(num == 1)
                    {
                        largest_x1 = points[i];
                        num++;
                    }
                    else if(num == 2)
                    {
                        largest_x2 = points[i];
                    }
                }
            }

            // Ok we now know the two points which have the largest X value.
            // To determine which indexes we need to put them at, we now turn our
            // attention to their Y values. We need to put the one with the larger
            // Y value at index 1, and the one with the smaller Y value at index 2
            // (see the graphic at the beginning of this method)
            if (largest_x1.y > largest_x2.y)
            {
                pointsSorted[1] = largest_x1;
                pointsSorted[2] = largest_x2;
            }
            else
            {
                pointsSorted[1] = largest_x2;
                pointsSorted[2] = largest_x1;
            }

            // We're all done. Return the points we sorted.
            return pointsSorted;
        }

        /**
         * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
         * original size of the tag.
         *
         * @param points the points which form the trapezoid
         * @param cameraMatrix the camera intrinsics matrix
         * @param tagsizeX the original width of the tag
         * @param tagsizeY the original height of the tag
         * @return the 6DOF pose of the camera relative to the tag
         */
        Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX , double tagsizeY)
        {
            // The actual 2d points of the tag detected in the image
            MatOfPoint2f points2d = new MatOfPoint2f(points);

            // The 3d points of the tag in an 'ideal projection'
            Point3[] arrayPoints3d = new Point3[4];
            arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
            arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
            arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
            arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
            MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

            // Using this information, actually solve for pose
            Pose pose = new Pose();
            Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

            return pose;
        }

        void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
        {
            //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
            //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

            // The points in 3D space we wish to project onto the 2D image plane.
            // The origin of the coordinate space is assumed to be in the center of the detection.
            MatOfPoint3f axis = new MatOfPoint3f(
                    new Point3(-tagWidth/2, tagHeight/2,0),
                    new Point3( tagWidth/2, tagHeight/2,0),
                    new Point3( tagWidth/2,-tagHeight/2,0),
                    new Point3(-tagWidth/2,-tagHeight/2,0),
                    new Point3(-tagWidth/2, tagHeight/2,-length),
                    new Point3( tagWidth/2, tagHeight/2,-length),
                    new Point3( tagWidth/2,-tagHeight/2,-length),
                    new Point3(-tagWidth/2,-tagHeight/2,-length));

            // Project those points
            MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
            Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
            Point[] projectedPoints = matProjectedPoints.toArray();

            // Pillars
            for(int i = 0; i < 4; i++)
            {
                Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], blue, 5);
            }

            // Base lines
            //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, 5);
            //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, 5);
            //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, 5);
            //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, 5);

            // Top lines
            Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, 5);
            Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, 5);
            Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, 5);
            Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, 5);
        }

        /**
         * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
         *
         * @param buf the RGB buffer on which to draw the marker
         * @param length the length of each of the marker 'poles'
         * @param rvec the rotation vector of the detection
         * @param tvec the translation vector of the detection
         * @param cameraMatrix the camera matrix used when finding the detection
         */
        void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
        {
            // The points in 3D space we wish to project onto the 2D image plane.
            // The origin of the coordinate space is assumed to be in the center of the detection.
            MatOfPoint3f axis = new MatOfPoint3f(
                    new Point3(0,0,0),
                    new Point3(length,0,0),
                    new Point3(0,length,0),
                    new Point3(0,0,-length)
            );

            // Project those points
            MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
            Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
            Point[] projectedPoints = matProjectedPoints.toArray();

            // Draw the marker!
            Imgproc.line(buf, projectedPoints[0], projectedPoints[1], new Scalar(255,0,0), thickness);
            Imgproc.line(buf, projectedPoints[0], projectedPoints[2], new Scalar(0,255,0), thickness);
            Imgproc.line(buf, projectedPoints[0], projectedPoints[3], new Scalar(0,0,255), thickness);
        }

        /**
         * Calculate the subpixel positions of the corners of a region. This allows
         * for getting more accurate corner positions than the integer rounding to
         * specific pixels would. It is very important to get accurate corner positions
         * when using the corners to extract 6DOF pose.
         *
         * @param buf the image on which to perform the subpixel corner search (MUST BE GREYSCALE!!)
         * @param corners the rough locations on the image of where the corners of the region are located
         * @param searchWindowPixels how many pixels the subpixel search algorithm is allowed to walk from the rough locations provided
         * @return the same corners passed in, but refined to be sub-pixel accurate.
         */
        Point[] calculateSubpixelCorners(Mat buf, Point[] corners, int searchWindowPixels)
        {
            // Construct a mat containing the raw corners
            Mat matOfCorners = new Mat(4, 2, CvType.CV_32F);
            double[] cornerData = new double[8];
            for(int i = 0; i < 4; i++)
            {
                cornerData[i*2] = corners[i].x;
                cornerData[i*2+1] = corners[i].y;
            }
            matOfCorners.put(0,0,cornerData);

            // Create some parameters for the search
            TermCriteria terminationCriteria = new TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, 40, 0.001);
            Size searchWindowSize = new Size(searchWindowPixels,searchWindowPixels); // in pixels
            Size zeroZone = new Size(-1,-1); // -1 means not used

            // Actually do the search
            Imgproc.cornerSubPix(buf, matOfCorners, searchWindowSize, zeroZone, terminationCriteria);

            // Pull out the results of the search
            Point[] refinedPoints = new Point[4];
            for(int i = 0; i < 4; i++)
            {
                refinedPoints[i] = new Point(matOfCorners.get(i,0)[0], matOfCorners.get(i,1)[0]);
            }
            return refinedPoints;
        }

        /**
         * Performs a trapezoid fit on a contour, using a previously
         * computed convex hull.
         *
         * @param convexHull the convex hull of the contour in question
         * @param contour the contour in question
         * @return the points of a trapezoid which has been fitted to the contour
         */
        Point[] doTrapezoidFit(MatOfInt convexHull, MatOfPoint contour)
        {
            // Compute the points of a standard rotated rect fit
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect minAreaRect = Imgproc.minAreaRect(contour2f);
            Point[] mar_points = new Point[4];
            minAreaRect.points(mar_points);

            // Get the actual points which form the convex hull
            ArrayList<Point> hullPoints = hull2Points(convexHull, contour);

            Point[] trapezoidPoints = new Point[4];

            // Now we actually do the trapezoid fit. We do this by iterating
            // through the points found on the rotated rect we fit earlier,
            // and then for each of those, iterating through all the convex
            // hull points to find the one which is closest, using the point
            // distance formula. {Dist = sqrt(x^2 + y^2)}
            for(int i = 0; i < 4; i++)
            {
                double dist = -1;
                Point aPoint = new Point();

                for(Point hp : hullPoints)
                {
                    // Compute the distance from this hull point to the min area rect point
                    double hypot = Math.hypot(mar_points[i].x-hp.x, mar_points[i].y-hp.y);

                    // Any time we see a point closer than one we had chosen before, we
                    // switch to that one. (Also if dist==-1 we switch since that means
                    // it's our first iteration around this loop)
                    if(dist == -1 || hypot < dist)
                    {
                        dist = hypot;
                        aPoint = hp;
                    }
                }

                // Ok we finished our search for one point; now we rinse and repeat for
                // the others.
                trapezoidPoints[i] = aPoint;
            }

            return trapezoidPoints;
        }

        /**
         * Gets the actual points for a convex hull (OpenCV's raw
         * convex hull return is just the indexes in the contour
         * the convex hull was computed for)
         *
         * @param hull the convex hull in question
         * @param contour the contour from which the hull was computed
         * @return the actual points on the convex hull
         */
        ArrayList<Point> hull2Points(MatOfInt hull, MatOfPoint contour)
        {
            List<Integer> indexes = hull.toList();
            ArrayList<Point> points = new ArrayList<>();
            for(Integer index : indexes)
            {
                points.add(contour.toList().get(index));
            }

            return points;
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
                rvec = new Mat();
                tvec = new Mat();
            }

            public Pose(Mat rvec, Mat tvec)
            {
                this.rvec = rvec;
                this.tvec = tvec;
            }
        }
    }
}

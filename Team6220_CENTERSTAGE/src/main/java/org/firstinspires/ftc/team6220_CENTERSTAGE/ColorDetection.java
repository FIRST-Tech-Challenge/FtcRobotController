package org.firstinspires.ftc.team6220_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


import java.util.ArrayList;
import java.util.List;
/*
This class is used in processing the camera feed to find colors during autonomous.
 */
public class ColorDetection
{
    // Initializes members
    int cameraMonitorViewId;
    LinearOpMode myOpMode;
    Mat hierarchy = new Mat();
    Mat image = new Mat();
    Mat output_image = new Mat();
    OpenCvCamera robotCamera;

    // Default values for color detection ranges
    Scalar minRange = new Scalar(0, 0, 0);
    Scalar maxRange = new Scalar(255, 255, 255);

    // Shows coordinates of detected object on the camera screen
    public double centerPosX = 0.0;
    public double centerPosY = 0.0;

    // Signifies the slices of the camera's view that the team prop could potentially be in
    public enum PropPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    //constructor, pass in opmode so we can access the camera
    public ColorDetection(LinearOpMode opMode){
        myOpMode = opMode;
    }

    // Internal method to set the ranges for color detection
    private void setRanges(Scalar minRange, Scalar maxRange) {
        this.minRange = minRange;
        this.maxRange = maxRange;
    }

    // Returns an enum corresponding to prop position
    public PropPosition returnZone() {
        if (centerPosX <= Constants.SLICE_DIV_1) {
            return PropPosition.LEFT;
        } else if (centerPosX <= Constants.SLICE_DIV_2) {
            return PropPosition.MIDDLE;
        } else {
            return PropPosition.RIGHT;
        }
    }

    public class ColorDetectionPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input)
        {
            // Copies the input Matrix to a secondary one
            input.copyTo(output_image);
            // blur the image to reduce the impact of noisy pixels
            Imgproc.GaussianBlur(input, image, new Size(7,7),0);

            // convert image to grayscale
            Imgproc.cvtColor(image, image, Imgproc.COLOR_RGB2HSV);

            // Void all pixels not in specified range
            Core.inRange(image, minRange, maxRange, image);

            // Creates a list to contain contours, then finds contours to put into that list.
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(image, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            // Finds the biggest contour
            if (contours.size() > 1) {
                double maxArea = 0;
                int maxAreaContourIndex = 0;
                for (int i = 0; i < contours.size(); i++) {
                    if (Imgproc.contourArea(contours.get(i)) > maxArea) {
                        maxArea = Imgproc.contourArea(contours.get(i));
                        // Identifier for the contour with the largest detected connected area
                        maxAreaContourIndex = i;
                    }
                }

                // Highlights the biggest connected area of colors that fit in the specified range
                Imgproc.drawContours(output_image, contours, maxAreaContourIndex, Constants.borderColors, 2, -1);

                // Specifies a rectangle outlining the detected element
                Rect boundingRect = Imgproc.boundingRect(contours.get(maxAreaContourIndex));

                // Calculation variables for readability, signifies the upper right coordinate of boundingRect
                double boundRightmostX = boundingRect.x + boundingRect.width;
                double boundUppermostY = boundingRect.y + boundingRect.height;

                // Sets center coordinates
                centerPosX = (int)boundingRect.width/2 + boundingRect.x;
                centerPosY = (int)boundingRect.height/2 + boundingRect.y;
                Point circlePoint = new Point((centerPosX), (centerPosY));

                // Draws a rectangle that signifies where the borders between slices are
                Imgproc.rectangle(output_image, new Point(Constants.SLICE_DIV_1, 0), new Point(Constants.SLICE_DIV_2, Constants.CAMERA_HEIGHT), Constants.borderColors, 4, Imgproc.LINE_8, 0);

                // Draws circle and rectangle, visibly indicating the detected team prop for use in programs such as scrcpy
                Imgproc.rectangle(output_image, new Point(boundingRect.x, boundingRect.y), new Point(boundRightmostX, boundUppermostY), Constants.borderColors, 1, Imgproc.LINE_8, 0);
                Imgproc.circle(output_image, circlePoint , 10, Constants.borderColors , Imgproc.LINE_8, -1);
            }
            // what opencv sees
            return output_image;
        }
    }



    public void init(Scalar minRange, Scalar maxRange) {

        cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                myOpMode.hardwareMap.appContext.getPackageName());
        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(myOpMode.hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);
        robotCamera.setPipeline(new ColorDetectionPipeline());

        setRanges(minRange, maxRange);

        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                robotCamera.startStreaming(Constants.CAMERA_WIDTH, Constants.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });

    }

}
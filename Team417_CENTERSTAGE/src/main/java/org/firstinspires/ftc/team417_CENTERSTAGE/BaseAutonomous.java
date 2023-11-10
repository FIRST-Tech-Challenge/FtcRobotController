package org.firstinspires.ftc.team417_CENTERSTAGE;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team417_CENTERSTAGE.opencv.OpenCvColorDetection;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
abstract public class BaseAutonomous extends BaseOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    int lastEncoderFL = 0;
    int lastEncoderFR = 0;
    int lastEncoderBL = 0;
    int lastEncoderBR = 0;

    // Prop detection variables
    Rect boundingRect;
    Point center;

    // Auton tuning constants
    public static double LEFT_Y = 26.0;
    public static double LEFT_X = -12.0;
    public static double RIGHT_Y = 26.0;
    public static double RIGHT_X = 12.0;
    public static double CENTER_Y = 29.0;
    public static double CENTER_X = 4.5;
    public static double INTAKE_SPEED = 1;
    public static double INTAKE_TIME = 3000; // in milliseconds

    public static double INTAKE_SPEED2 = 1;
    public static double INTAKE_TIME2 = 10000; // in milliseconds
    public static double MOVING_FROM_WALL = 3.0;
    public static double FAR_PARKING = 96;
    public static double CLOSE_PARKING = 48;
    public static double ROBOT_SPEED = 0.5;
    public static double STRAFE_FACTOR = 1.210719915922228;
    public static double DISTANCE_FACTOR = 1.032258064516129;

    public static double Y_CALIBRATION_RIGHT = -2.0;
    public static double Y_CALIBRATION_LEFT = 1.0;


    public void driveInches(double x, double y) {
        double xTicks = x * TICKS_PER_INCH * STRAFE_FACTOR;
        double yTicks = y * TICKS_PER_INCH * DISTANCE_FACTOR;

        double targetFL = xTicks + yTicks;
        double targetFR = yTicks - xTicks;
        double targetBL = yTicks - xTicks;
        double targetBR = yTicks + xTicks;

        // Determine new target position, and pass to motor controller
        targetFL += FL.getCurrentPosition();
        targetFR += FR.getCurrentPosition();
        targetBL += BL.getCurrentPosition();
        targetBR += BR.getCurrentPosition();

        FL.setTargetPosition((int) targetFL);
        FR.setTargetPosition((int) targetFR);
        BL.setTargetPosition((int) targetBL);
        BR.setTargetPosition((int) targetBR);

        // Turn On RUN_TO_POSITION
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        FL.setPower(ROBOT_SPEED);
        FR.setPower(ROBOT_SPEED);
        BL.setPower(ROBOT_SPEED);
        BR.setPower(ROBOT_SPEED);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (runtime.seconds() < 30) &&
                (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) {
        }

        // Stop all motion;
        stopDriving();

        // Turn off RUN_TO_POSITION
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    OpenCvColorDetection myColorDetection = new OpenCvColorDetection(this);

    public void initializeAuto() {
        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        initializeHardware();
        myColorDetection.init();

        telemetry.addData("Init State", "Init Finished");

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));

        // cameraMonitorViewId allows us to see the image pipeline using scrcpy
        //   for easy debugging
        //   You can disable it after testing completes
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);


        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);


        //camera.setPipeline(new PropDetectionPipeline());

        /*
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH_PIXELS, CAMERA_HEIGHT_PIXELS, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        */

        // Set last know encoder values
        lastEncoderFR = FR.getCurrentPosition();
        lastEncoderFL = FL.getCurrentPosition();
        lastEncoderBL = BL.getCurrentPosition();
        lastEncoderBR = BR.getCurrentPosition();

        sleep(5000);

        telemetry.clear();
        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();
    }

    public static final Scalar LOWER_BLUE = new Scalar(100, 150, 100);
    public static final Scalar UPPER_BLUE = new Scalar(130, 255, 255);

    public static final Scalar LOWER_RED = new Scalar(10, 150, 100);
    public static final Scalar UPPER_RED = new Scalar(40, 255, 255);

    boolean detectingBlue;

    class PropDetectionPipeline extends OpenCvPipeline {
        boolean viewportPaused = false;
        Mat hsv = new Mat();
        Mat output = new Mat();
        Mat resizedMask = new Mat();
        Mat hierarchy = new Mat();
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        MatOfPoint largestContour = new MatOfPoint();
        Size gaussianBlurSize = new Size(7, 7);
        Size binaryMaskSize = new Size(0, 0);
        Point center = new Point();
        Scalar markingColor = new Scalar(0, 255, 0);


        @Override
        public Mat processFrame(Mat input) {
            int rows = input.rows();
            int cols = input.cols();

            // calculate the new top of region
            int newTop = rows / 2;

            // crop image to region of interest
            input.adjustROI(newTop, cols + 1, 0, rows + 1);

            // convert image to hsv
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // blur the image to reduce the impact of noisy pixels
            Imgproc.GaussianBlur(hsv, hsv, gaussianBlurSize, 0);
            if (detectingBlue) {
                Core.inRange(hsv, LOWER_BLUE, UPPER_BLUE, hsv);
            } else {
                Core.inRange(hsv, LOWER_RED, UPPER_RED, hsv);
            }
            Imgproc.threshold(hsv, output, 1, 255, Imgproc.THRESH_BINARY);
            // Resize the binary mask
            Imgproc.resize(output, resizedMask, binaryMaskSize, 1, 1, Imgproc.INTER_AREA);

            // Find contours
            Imgproc.findContours(resizedMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            if (contours.size() > 1) {
                // Find the largest contour
                double maxArea = -1;
                int maxAreaIndex = -1;
                for (int i = 0; i < contours.size(); i++) {
                    double area = Imgproc.contourArea(contours.get(i));
                    if (area > maxArea) {
                        maxArea = area;
                        maxAreaIndex = i;
                    }
                }

                largestContour = contours.get(maxAreaIndex);
                boundingRect = Imgproc.boundingRect(largestContour);
                // Draw a rectangle around the largest contour on the frame
                Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), markingColor, 2);

                // Calculate the center of the bounding rectangle
                center.x = boundingRect.x + (boundingRect.width * 0.5);
                center.y = boundingRect.y + (boundingRect.height * 0.5);

                // Draw the largest contour
                Imgproc.drawContours(input, contours, maxAreaIndex, markingColor);

                // Draw a circle at the center
                Imgproc.circle(input, center, 2, markingColor, 2);

                int width = input.width();
                int contourX = (int) center.x;

                // Determine position based on the X coordinate of the center


                largestContour.release();
            }
            resizedMask.release();
            hierarchy.release();

            return input;
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                camera.pauseViewport();
            } else {
                camera.resumeViewport();
            }
        }
    }

    private void stopDriving() {
        FL.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        BR.setPower(0.0);
    }

    public void runSimpleInchesAuto(boolean red, boolean close) {
        if (red) {
            myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.RED);
            //telemetry.addData("Blue")
        } else {
            myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.BLUE);
        }

        initializeAuto();

        waitForStart();
        Log.d("skid", toString());

        double x = 0;
        double y = 0;
        switch (myColorDetection.detectTeamProp()) {
            case LEFT:
                telemetry.addData("Side", "Left");
                x = LEFT_X;
                y = LEFT_Y;
                break;
            case CENTER:
                telemetry.addData("Side", "Center");
                x = CENTER_X;
                y = CENTER_Y;
                break;
            case RIGHT:
                telemetry.addData("Side", "Right");
                x = RIGHT_X;
                y = RIGHT_Y;
                break;
            default:
                telemetry.addData("Side", "Unsure");
        }
        telemetry.update();
        /* driveInches(0, y);
        driveInches(x, 0);

        if(intakeMotor != null) {
              intakeMotor.setPower(INTAKE_SPEED);
              sleep((long) INTAKE_TIME);
              intakeMotor.setPower(0);
        } else {
            sleep(5000); 
        }

        driveInches(-x, 0);
        driveInches(0, -y);
        */
        driveInches(0, MOVING_FROM_WALL);

        if (close) {
            if (red) {
                driveInches(CLOSE_PARKING, Y_CALIBRATION_RIGHT);
            } else {
                driveInches(-CLOSE_PARKING, Y_CALIBRATION_LEFT);
            }
        } else {
            if (red) {
                driveInches(FAR_PARKING, Y_CALIBRATION_RIGHT);
            } else {
                driveInches(-FAR_PARKING, Y_CALIBRATION_LEFT);
            }
        }

        if (intakeMotor != null) {
            intakeMotor.setPower(-INTAKE_SPEED2);
            sleep((long) INTAKE_TIME2);
            intakeMotor.setPower(0);
        } else {
            sleep(5000);
        }
    }

    @Override
    public String toString() {
        return "BaseAutonomous{" +
                "runtime=" + runtime +
                ", lastEncoderFL=" + lastEncoderFL +
                ", lastEncoderFR=" + lastEncoderFR +
                ", lastEncoderBL=" + lastEncoderBL +
                ", lastEncoderBR=" + lastEncoderBR +
                ", LEFT_Y=" + LEFT_Y +
                ", LEFT_X=" + LEFT_X +
                ", RIGHT_Y=" + RIGHT_Y +
                ", RIGHT_X=" + RIGHT_X +
                ", CENTER_Y=" + CENTER_Y +
                ", CENTER_X=" + CENTER_X +
                ", INTAKE_SPEED=" + INTAKE_SPEED +
                ", INTAKE_TIME=" + INTAKE_TIME +
                ", MOVING_FROM_WALL=" + MOVING_FROM_WALL +
                ", FAR_PARKING=" + FAR_PARKING +
                ", CLOSE_PARKING=" + CLOSE_PARKING +
                ", ROBOT_SPEED=" + ROBOT_SPEED +
                ", STRAFE_FACTOR=" + STRAFE_FACTOR +
                ", DISTANCE_FACTOR=" + DISTANCE_FACTOR +
                ", LOWER_BLUE=" + LOWER_BLUE +
                ", UPPER_BLUE=" + UPPER_BLUE +
                ", LOWER_RED=" + LOWER_RED +
                ", UPPER_RED=" + UPPER_RED +
                ", detectingBlue=" + detectingBlue +
                ", sideDetected=" + myColorDetection.sideDetected +
                '}';
    }
}

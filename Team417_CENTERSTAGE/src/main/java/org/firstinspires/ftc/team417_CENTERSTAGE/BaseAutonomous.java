package org.firstinspires.ftc.team417_CENTERSTAGE;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveByTime_Linear;
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
    public static double INTAKE_SPEED = -0.5;
    public static double INTAKE_TIME = 2000; // in milliseconds

    public static double INTAKE_SPEED2 = -0.5;
    public static double INTAKE_TIME2 = 2000; // in milliseconds
    public static double MOVING_FROM_WALL = 3;
    public static double FAR_PARKING = 96;
    public static double CLOSE_PARKING = 48;
    public static double ROBOT_SPEED = 0.5;
    public static double STRAFE_FACTOR = 1.210719915922228;
    public static double DISTANCE_FACTOR = 1.032258064516129;


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

    private int CAMERA_WIDTH_PIXELS = 640;
    private int CAMERA_HEIGHT_PIXELS = 480;

    public void initializeAuto() {
        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        initializeHardware();

        telemetry.addData("Init State", "Init Finished");

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        camera.setPipeline(new PropDetectionPipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH_PIXELS, CAMERA_HEIGHT_PIXELS, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

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

    public static final Scalar LOWER_BLUE_OR_RED = new Scalar(100, 150, 100);
    public static final Scalar UPPER_BLUE_OR_RED = new Scalar(130, 255, 255);

    boolean detectingBlue;

    enum SideDetected {
        INITIALIZED,
        LEFT,
        CENTER,
        RIGHT
    }

    public SideDetected sideDetected = SideDetected.INITIALIZED;

    class PropDetectionPipeline extends OpenCvPipeline {
        boolean viewportPaused = false;
        Mat hsv = new Mat();
        Mat output = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            // convert image to grayscale
            if (detectingBlue) {
                Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            } else {
                Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
            }
            // blur the image to reduce the impact of noisy pixels
            Imgproc.GaussianBlur(hsv, hsv, new Size(7, 7), 0);
            Core.inRange(hsv, LOWER_BLUE_OR_RED, UPPER_BLUE_OR_RED, hsv);
            Imgproc.threshold(hsv, output, 1, 255, Imgproc.THRESH_BINARY);
            // Resize the binary mask
            Mat resizedMask = new Mat();
            Imgproc.resize(output, resizedMask, new Size(0, 0), 1, 1, Imgproc.INTER_AREA);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(resizedMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            if (contours.size() > 0) {
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

                MatOfPoint largestContour = contours.get(maxAreaIndex);
                boundingRect = Imgproc.boundingRect(largestContour);
                // Draw a rectangle around the largest contour on the frame
                Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(0, 255, 0), 2);

                // Calculate the center of the bounding rectangle
                center = new Point(boundingRect.x + (boundingRect.width * 0.5), boundingRect.y + (boundingRect.height * 0.5));

                // Draw the largest contour
                Imgproc.drawContours(input, contours, maxAreaIndex, new Scalar(0, 255, 0));

                // Draw a circle at the center
                Imgproc.circle(input, center, 2, new Scalar(0, 255, 0), 2);

                int width = input.width();
                int contourX = (int) center.x;

                // Determine position based on the X coordinate of the center
                if (contourX < width / 3) {
                    sideDetected = SideDetected.LEFT;
                } else if (contourX > (2 * width) / 3) {
                    sideDetected = SideDetected.RIGHT;
                } else {
                    sideDetected = SideDetected.CENTER;
                }
            }
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

    public SideDetected detectTeamProp() {
        Log.d("skid prop", sideDetected + " " + boundingRect.toString() + " " + center.toString());
        return sideDetected;
    }

    private void stopDriving() {
        FL.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        BR.setPower(0.0);
    }

    public void runSimpleInchesAuto(boolean red, boolean close) {
        initializeAuto();
        waitForStart();
        Log.d("skid", toString());

        if (red == false) {
            detectingBlue = true;
            //telemetry.addData("Blue")
        } else {
            detectingBlue = false;
        }

        double x = 0;
        double y = 0;
        switch (detectTeamProp()) {
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
        driveInches(0, y);
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
        driveInches(0, MOVING_FROM_WALL);

        if (close) {
            if (red) {
                driveInches(CLOSE_PARKING, 0);
            } else {
                driveInches(-CLOSE_PARKING, 0);
            }
        } else {
            if (red) {
                driveInches(FAR_PARKING,0);
            } else {
                driveInches(-FAR_PARKING, 0);
            }
        }

        if(intakeMotor != null) {
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
                ", CAMERA_WIDTH_PIXELS=" + CAMERA_WIDTH_PIXELS +
                ", CAMERA_HEIGHT_PIXELS=" + CAMERA_HEIGHT_PIXELS +
                ", LOWER_BLUE_OR_RED=" + LOWER_BLUE_OR_RED +
                ", UPPER_BLUE_OR_RED=" + UPPER_BLUE_OR_RED +
                ", detectingBlue=" + detectingBlue +
                ", sideDetected=" + sideDetected +
                '}';
    }
}

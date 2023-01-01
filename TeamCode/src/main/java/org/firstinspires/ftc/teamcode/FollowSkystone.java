package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Follow Skystone")
//@Disabled
public class FollowSkystone extends LinearOpMode {
    OpenCvCamera webcam;
    //TODO: Insert correct Hardware2 class and instantiate robot
    TestHardware robot = new TestHardware(false);
    BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.initTeleOpIMU(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        //parameters.angleUnit = BNO055IMU.AngleUnit.??????; //unit for turning
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        while(!imu.isGyroCalibrated()){

        }
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Open the connection to the camera device
         */
        webcam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        SkystoneDetectionPipeline pipeline = new SkystoneDetectionPipeline();
        webcam.setPipeline(pipeline);

        /*
         * Tell the webcam to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
         * supports streaming from the webcam in the uncompressed YUV image format. This means
         * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
         * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
         *
         * Also, we specify the rotation that the webcam is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive()) {
            /*
             * Send some stats to the telemetry
             */

            double leftpow = 0;
            double rightpow = 0;

            if (pipeline.getContourArea() > 11000){
                leftpow = -0.2;
                rightpow = -0.2;
            }
            if (pipeline.getContourArea() < 9000){
                leftpow = 0.2;
                rightpow = 0.2;
            }

            if (pipeline.getTargetLocation().x > 180){
                leftpow += 0.2;
                rightpow -= 0.2;
            }

            if (pipeline.getTargetLocation().x < 140){
                leftpow -= 0.2;
                rightpow += 0.2;
            }

            robot.setAllMotors(leftpow, rightpow);

            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Center of Contour", pipeline.getTargetLocation());
            telemetry.addData("Contour Area", pipeline.getContourArea());
            telemetry.update();
        }
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    public void turnWithPID(double targetAngle, double kp, double ki, double kd, double threshold){
        //setting up values
        double error = angleWrap(targetAngle - getHeading());
        //sum value for the integral constant
        double sum = error;
        double previousError = error;
        //value that gets passed to the motors
        double correction = 0;
        //slope value for the derivative constant
        double slope = 0;
        //setting up timer
        runtime.reset();
        while (Math.abs(error) > threshold && opModeIsActive()){
            //sampling how far robot needs to turn
            error = angleWrap(targetAngle - getHeading());
            //calculating slope
            slope = (error - previousError)/(double) runtime.time();
            runtime.reset();
            sum += error;
            //calculating motor powers
            correction = kp * error + ki * sum + kd * slope;
            //TODO: This function was specific to our group. When implementing this function, set left motors to (-correction) and right motors to (correction)
            //robot.setAllMotors(-1 * correction, correction);
        }
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        return heading;
    }

    public double angleWrap(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }
}

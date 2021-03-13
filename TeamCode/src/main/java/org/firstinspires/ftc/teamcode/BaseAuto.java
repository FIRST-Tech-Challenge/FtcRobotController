package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


public abstract class BaseAuto extends LinearOpMode {

    HardwareMapV2 robot = null;

    ElapsedTime runtime = new ElapsedTime();

    // define and initialize variables

    static final double     COUNTS_PER_MOTOR_REV    = 767.2 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 10.0 ;     // This measurement is more exact than inches
    static final double     COUNTS_PER_CM         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI));
    static final double     DRIVE_SPEED             = 0.9;
    static final double     TURBO_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.3;
    static final double     P_TURN_COEFF            = 0.03;
    static final double     HEADING_THRESHOLD       = 1 ;
    static final double     P_DRIVE_COEFF           = 0.03;



    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;




    static final double  BRAKING_DISTANCE_COUNTS  = (40 * COUNTS_PER_CM);
    static final double  ACCELERATING_DISTANCE_COUNTS  = (20 * COUNTS_PER_CM);

    SkystoneDeterminationPipeline pipeline;

    BNO055IMU imu;


    // initializes hardware  and vuforia; calibrates gyro
    public void inithardware(){

        HardwareMapV2 robot = new HardwareMapV2();

        robot.init(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

//        // Send telemetry message to alert driver that we are calibrating;
//        telemetry.addData("Calibrating Gyro:", "Dont do anything");    //
//        telemetry.update();
//
//        while (!isStopRequested() && !imu.isGyroCalibrated())
//        {
//            sleep(50);
//            idle();
//        }

        telemetry.addData("Calibrating Gyro: ", "Wait 3 seconds");    //
        telemetry.update();

        sleep(3000);

        telemetry.addData("Ready! ", "Let's go");    //
        telemetry.update();



    }
    // NOT USED; gets average gyro value for more accurate angles
    public double getAverageGyro(){
        /*int sum = robot.realgyro.getIntegratedZValue() + robot.realgyro2.getIntegratedZValue();
        return sum/2;*/
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);
        double heading = angles.firstAngle;
        return heading;
    }


/*    // Only for mecanum drive; uses gyro to move robot at a specific angle and distance (cm)
    public void gyroMecanumDrive ( double speed, double distance, double angle) {

        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;

        int     moveCounts;
        double  targetAvg;
        double  currentAvg;
        double  initialAvg;
        double  remainingCounts;
        double  countsTraveled;
        double  factor;

        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            factor = 1;
            moveCounts = (int)(distance * COUNTS_PER_CM);
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + moveCounts;
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + moveCounts;
            newBackLeftTarget = robot.backLeft.getCurrentPosition() + moveCounts;
            newBackRightTarget = robot.backRight.getCurrentPosition() + moveCounts;
            targetAvg = (newBackLeftTarget + newBackRightTarget + newFrontLeftTarget + newFrontRightTarget)/4;
            currentAvg = (robot.frontLeft.getCurrentPosition()+robot.frontRight.getCurrentPosition()+ robot.backLeft.getCurrentPosition()+robot.backRight.getCurrentPosition())/4 ;
            initialAvg = currentAvg;

            // Set Target and Turn On RUN_TO_POSITION
            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.backRight.setTargetPosition(newBackRightTarget);
            robot.backLeft.setTargetPosition(newBackLeftTarget);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.frontLeft.setPower(speed);
            robot.frontRight.setPower(speed);
            robot.backRight.setPower(speed);
            robot.backLeft.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())  ) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
                currentAvg = (robot.frontLeft.getCurrentPosition()+robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition())/4 ;
                remainingCounts = targetAvg - currentAvg;
                countsTraveled = currentAvg - initialAvg;


                if (countsTraveled < ACCELERATING_DISTANCE_COUNTS) {
                    factor = (countsTraveled * (1.0/ACCELERATING_DISTANCE_COUNTS)) + 0.1;
                    leftSpeed *= factor;
                    rightSpeed *= factor;
                    telemetry.addData("factor happened",leftSpeed);
                    telemetry.update();
                }

                if (remainingCounts <= BRAKING_DISTANCE_COUNTS) {
                    factor = remainingCounts * (1.0/BRAKING_DISTANCE_COUNTS);
                    leftSpeed *= factor;
                    rightSpeed *= factor;
                    telemetry.addData("factor happened",leftSpeed);
                    telemetry.update();
                }


                robot.frontLeft.setPower(leftSpeed);
                robot.frontRight.setPower(rightSpeed);


                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.addData("TargetAvg",targetAvg);
                telemetry.addData("CurrentAvg",currentAvg);
                telemetry.addData("factor",factor);



                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);



            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

*/

    // checks to make sure that all motors which should be running are running
    public boolean areMotorsRunning(MecanumWheels wheels){
        for (int i = 0; i < wheels.wheelPowers.length;i++){
            if (wheels.wheelPowers[i]!=0){
                switch (i){
                    case 0:
                        if (!robot.frontLeft.isBusy()) {
                            telemetry.addData("front left motor", "done");
                            telemetry.update();

                            return false;
                        }
                        break;
                    case 1:
                        if (!robot.frontRight.isBusy()) {
                            telemetry.addData("front right power",  wheels.wheelPowers[i]);
                            telemetry.addData("front right motor", "done");
                            telemetry.update();

                            return false;
                        }
                        break;
                    case 2:
                        if (!robot.backLeft.isBusy()){
                            telemetry.addData("back left motor", "done");
                            telemetry.update();

                            return false;
                        }
                        break;
                    case 3:
                        if (!robot.backRight.isBusy()) {
                            telemetry.addData("back right motor", "done");
                            telemetry.update();

                            return false;
                        }
                        break;
                }
            }
        }
        return true;
    }


    // simple directional drive function for a mecanum drive train
    public void encoderMecanumDrive(double speed, double distance , double timeoutS, double move_x, double move_y) {
        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     frontLeftSign;
        int     frontRightSign;
        int     backLeftSign;
        int     backRightSign;
        MecanumWheels wheels = new MecanumWheels();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            wheels.UpdateInput(move_x, move_y, 0);


            frontLeftSign = (int) (Math.abs(wheels.getFrontLeftPower())/wheels.getFrontLeftPower());
            frontRightSign = (int) (Math.abs(wheels.getFrontRightPower())/wheels.getFrontRightPower());
            backLeftSign = (int) (Math.abs(wheels.getRearLeftPower()) /wheels.getRearLeftPower());
            backRightSign = (int) (Math.abs(wheels.getRearRightPower())/wheels.getRearRightPower());



            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*frontLeftSign);
            newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*backLeftSign);
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*frontRightSign);
            newBackRightTarget = robot.backRight.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*backRightSign);


            //Set target position
            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.backLeft.setTargetPosition(newBackLeftTarget);
            robot.backRight.setTargetPosition(newBackRightTarget);



            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(wheels.getFrontLeftPower()*speed));
            robot.frontRight.setPower(Math.abs(wheels.getFrontRightPower()*speed));
            robot.backRight.setPower(Math.abs(wheels.getRearRightPower()*speed));
            robot.backLeft.setPower(Math.abs(wheels.getRearLeftPower()*speed));



            while (opModeIsActive() && (runtime.seconds() < timeoutS) && areMotorsRunning(wheels)) {

                telemetry.addData("FrontLeftPower",robot.frontLeft.getPower());
                telemetry.addData("FrontRightPower",robot.frontRight.getPower());
                telemetry.addData("BackRightPower",robot.backRight.getPower());
                telemetry.addData("BackLeftPower",robot.backLeft.getPower());
//                telemetry.addData("front left power",  wheels.wheelPowers[0]);
//                telemetry.addData("back left power",  wheels.wheelPowers[2]);
//                telemetry.addData("back right power",  wheels.wheelPowers[3]);
//                telemetry.addData("front right power",  wheels.wheelPowers[1]);
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    // simple directional drive function for a mecanum drive train
    public void testdrive(double speed, double distance, DcMotor input ) {

        int newTarget;

        MecanumWheels wheels = new MecanumWheels();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            wheels.UpdateInput(0, 1, 0);

            // Determine new target position, and pass to motor controller

            newTarget = input.getCurrentPosition() + (int)(distance * COUNTS_PER_CM);

            input.setTargetPosition(newTarget);

            // reset the timeout time and start motion.
            runtime.reset();

            input.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            input.setPower(wheels.getRearRightPower()*speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive()  && (input.isBusy())) {

                // Display it for the driver.
                //         telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                //       telemetry.addData("Path2",  "Running at %7d :%7d",

                telemetry.addData("BackRightPower",input.getPower());
                telemetry.addData("current position",input.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            input.setPower(0);



            // Turn off RUN_TO_POSITION
            input.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


/*    // NOT USED; uses encoders to drive - no gyro - tank drive
    public void encoderDrive(double speed, double leftCm, double rightCm, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(leftCm * COUNTS_PER_CM);
            newRightTarget = robot.frontRight.getCurrentPosition() + (int)(rightCm * COUNTS_PER_CM);
            robot.frontLeft.setTargetPosition(newLeftTarget);
            robot.frontRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(speed));
            robot.frontRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
*/
    // turn to an angle using gyro
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("current_heading", getAverageGyro());
            telemetry.update();
        }
    }

    // checks distance for gyroTurn/Drive - slows down when closer to end
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.frontLeft.setPower(leftSpeed);
        robot.frontRight.setPower(rightSpeed);
        robot.backLeft.setPower(leftSpeed);
        robot.backRight.setPower(rightSpeed);


        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    //show the error in gyro angle
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getAverageGyro();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    public void initializeOpenCV() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        final OpenCvCamera phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(165,63);

        static final int REGION_WIDTH = 95;
        static final int REGION_HEIGHT = 80;

        final int FOUR_RING_THRESHOLD = 137;
        final int ONE_RING_THRESHOLD = 130;

//        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,0);
//
//        static final int REGION_WIDTH = 320;
//        static final int REGION_HEIGHT = 240;
//
//        final int FOUR_RING_THRESHOLD = 150;
//        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 >= FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 >= ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

}


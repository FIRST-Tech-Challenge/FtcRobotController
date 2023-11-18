package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous
public class CenterStageAutoBlueBack extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    private TfodProcessor tfod;
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor carWashMotor = null;
    private IMU imu = null;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = (double) ((((1+(46.0/17))) * (1+(46.0/11))) * 28) ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415926535897932384626433832795028841);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    double carWashPower = 1.0;

    private VisionPortal visionPortal;

    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        carWashMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");

        initTfod();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        // Main code
        //dropCarWash();
        drive(13.5);
        //drive(1);
        //*
        List<Recognition> pixels = telemetryTfod();
        if (pixels.size() > 0) {
            drive(6);
            ejectPixel();
            drive(-6);
        } else {
            turn(-30);
            pixels = telemetryTfod();
            if (pixels.size() > 0) {
                drive(6);
                ejectPixel();
                drive(-6);
                turn(30);
            } else {
                turn(60);
                drive(6);
                ejectPixel();
                drive(-6);
                turn(-30);
            }
        }
        //*/
        turn(-60);
        drive(20);
        ejectPixel();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftBackDrive.isBusy() && rightBackDrive.isBusy() && leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

            //  stopRobot();

            // Turn off RUN_TO_POSITION
            // Note: Following code is technically redundant since called in stopRobot(), but the function
            // may be changed, so do not delete.
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move.
        }
    }

    public void turn(double degrees) {
        double TURN_ACCURACY = 1.5;
        if (false) { // Boolean determines the method the robot takes to turn x degrees
            encoderDrive(TURN_SPEED, degrees / 7.5, -degrees / 7.5, degrees / 36);
        } else {
            degrees *= -1;
            double startAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while (imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle < degrees - TURN_ACCURACY || imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle > degrees + TURN_ACCURACY) {
                leftBackDrive.setPower(-degrees / Math.abs(degrees) * TURN_SPEED);
                rightBackDrive.setPower(degrees / Math.abs(degrees) * TURN_SPEED);
                leftFrontDrive.setPower(-degrees / Math.abs(degrees) * TURN_SPEED);
                rightFrontDrive.setPower(degrees / Math.abs(degrees) * TURN_SPEED);
            }
            stopRobot();
        }
    }

    public void drive(double inches) {
        double startAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int checks = 1; // Number of times the robot will check its orientation during a single drive movement and correct itself
        for(int i = 0; i < checks; i++) {
            encoderDrive(DRIVE_SPEED, inches / checks, inches / checks, inches / checks / 4 + 1);
            //turn(startAngle - imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        }
        stopRobot();
    }

    public void ejectPixel() {
        int t = (int) runtime.milliseconds() + 1000;
        carWashMotor.setPower(carWashPower);
        while(true) {
            if (!(t > ((int) runtime.milliseconds()))){
                break;
            }
        }
        carWashMotor.setPower(0);
    }

    public void stopRobot() {
        // Turn On RUN_TO_POSITION
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Stop all motion
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition());
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition());
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition());
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition());

        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void dropCarWash() {
        drive(15);
        drive(-15);
        sleep(100);
    }

    public void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    }

    public List<Recognition> telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        } // Telemetry
        return currentRecognitions;
    }
}
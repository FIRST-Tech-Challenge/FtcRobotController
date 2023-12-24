package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.signum;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

@Autonomous(name = "Turning",group = "Tests")
public class TurningTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    private TfodProcessor tfod;
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx lf = null;
    private DcMotorEx lb = null;
    private DcMotorEx rf = null;
    private DcMotorEx rb = null;
    private DcMotorEx carWashMotor = null;
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
    static final double     PI                      = 3.141592653589793238462643383279502884169399;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 500;
    double carWashPower = 1.0;
    double offset = 0;

    private VisionPortal visionPortal;

    public void runOpMode() {
        // Initialize the drive system variables.
        lf = hardwareMap.get(DcMotorEx.class, "leftFront");
        lb = hardwareMap.get(DcMotorEx.class, "leftBack");
        rf = hardwareMap.get(DcMotorEx.class, "rightFront");
        rb = hardwareMap.get(DcMotorEx.class, "rightBack");
        imu = hardwareMap.get(IMU.class, "imu");

        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.REVERSE);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lb.setTargetPosition(lb.getCurrentPosition());
        rb.setTargetPosition(rb.getCurrentPosition());
        lf.setTargetPosition(lf.getCurrentPosition());
        rf.setTargetPosition(rf.getCurrentPosition());

        lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        initTfod();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        float programStartAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // Main code
        turn(90);
        sleep(15000);
        while (opModeIsActive()) {
            telemetry.addData("Status", "Done");
            telemetry.addData("Original Angle", programStartAngle);
            telemetry.addData("Angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
        }


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
        int lbTarget;
        int rbTarget;
        int lfTarget;
        int rfTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            lbTarget = lb.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            rbTarget = rb.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            lfTarget = lf.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            rfTarget = rf.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            lb.setTargetPosition(lbTarget);
            rb.setTargetPosition(rbTarget);
            lf.setTargetPosition(lfTarget);
            rf.setTargetPosition(rfTarget);

            // Turn On RUN_TO_POSITION
            lb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lf.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rf.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            lb.setPower(abs(speed));
            rb.setPower(abs(speed));
            lf.setPower(abs(speed));
            rf.setPower(abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (lb.isBusy() && rb.isBusy() && lf.isBusy() && rf.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("Running to",  " %7d :%7d", lfTarget,  rfTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", lf.getCurrentPosition(), rf.getCurrentPosition());
                telemetry.update();
            }

            //  stopRobot();

            // Turn off RUN_TO_POSITION
            // Note: Following code is technically redundant since called in stopRobot(), but the function
            // may be changed, so do not delete.
            lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move.
        }
    }

    public void resetIMU() {
        sleep(250);
        offset = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public double getAngle() {
        return fixAngle(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }

    public double fixAngle(double angle) {
        if (abs(angle) > 180) {
            return (angle % 180) - 180 * signum(angle);
        } else {
            return angle;
        }
    }

    public double fixError(double error) {
        if (abs(error) > 180) {
            return (error % 180) - 180;
        } else {
            return error;
        }
    }

    public double closestToZero(double a, double b) {
        if (abs(a) < abs(b)) {
            return a;
        } else {
            return b;
        }
    }

    public void turn(double degrees) {
        if (false) { // Boolean determines the method the robot takes to turn x degrees
            encoderDrive(TURN_SPEED, degrees / 7.5, -degrees / 7.5, abs(degrees) / 36);
            stopRobot();
        } else if (true) {
            resetIMU();
            double tolerance = 5;
            degrees *= -1;
            double sign = signum(degrees);
            double startAngle = getAngle();
            double currentAngle;
            double initialGoalAngle = startAngle + degrees;
            double correctedGoalAngle = fixAngle(initialGoalAngle);
            double error = sign * 999;
            double turnModifier;
            double turnPower;
            double lastAngle = startAngle;
            while (opModeIsActive() && (sign * error > tolerance)) {
                currentAngle = getAngle();
                error = fixError(closestToZero(initialGoalAngle - currentAngle, correctedGoalAngle - currentAngle));
                turnModifier = min(1, abs((error + 3) / 45));
                turnPower = degrees / abs(degrees) * TURN_SPEED * turnModifier;
                lb.setVelocity(-turnPower);
                rb.setVelocity(turnPower);
                lf.setVelocity(-turnPower);
                rf.setVelocity(turnPower);
                telemetry.addData("Delta Angle", lastAngle - currentAngle);
                telemetry.addData("Corrected Goal", correctedGoalAngle);
                telemetry.addData("Initial Goal", initialGoalAngle);
                telemetry.addData("Start", startAngle);
                telemetry.addData("Angle", currentAngle);
                telemetry.addData("True Angle", currentAngle + offset);
                telemetry.addData("Turn Modifier", turnModifier);
                telemetry.addData("Raw error", error);
                telemetry.update();
                lastAngle = currentAngle;
            }
            stopRobot();
        } else {

        }
    }

    public void drive(double inches) {
        double startAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int checks = 1; // Number of times the robot will check its orientation during a single drive movement and correct itself
        for(int i = 0; i < checks; i++) {
            encoderDrive(DRIVE_SPEED, inches / checks, inches / checks, abs(inches) / checks / 4 + 1);
            //turn(startAngle - imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        }
        stopRobot();
    }

    public void ejectPixel() {/*
        int t = (int) runtime.milliseconds() + 1000;
        carWashMotor.setPower(carWashPower);
        while(true) {
            if (!(t > ((int) runtime.milliseconds()))){
                break;
            }
        }
        carWashMotor.setPower(0);//*/
    }

    public void stopRobot() {
        // Turn On RUN_TO_POSITION
        lb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Stop all motion
        lb.setTargetPosition(lb.getCurrentPosition());
        rb.setTargetPosition(rb.getCurrentPosition());
        lf.setTargetPosition(lf.getCurrentPosition());
        rf.setTargetPosition(rf.getCurrentPosition());

        lb.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        rf.setPower(0);

        // Turn off RUN_TO_POSITION
        lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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
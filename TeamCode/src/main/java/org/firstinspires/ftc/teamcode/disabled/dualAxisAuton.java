package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

// Taken from last year's MecanumDriving class

@Autonomous(name = "dualAxisAutonTest", group = "Zippo")
@Disabled

public class dualAxisAuton extends LinearOpMode {

    /* Declare OpMode members. */
    testPlatformHardware robot = new testPlatformHardware();
    private ElapsedTime runtime = new ElapsedTime();
    String xyz = "z";


    //MOTOR CONSTANTS
    static final double COUNTS_PER_MOTOR_REV = testPlatformHardware.COUNTS_PER_MOTOR_REV; //216
    static final double DRIVE_GEAR_REDUCTION = testPlatformHardware.DRIVE_GEAR_REDUCTION;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = testPlatformHardware.WHEEL_DIAMETER_INCHES;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    //static final double COUNTS_PER_INCH = 20;
    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 0.5;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    public double originalAngle;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    VuforiaTrackables targetsSkyStone;
    VuforiaTrackable stoneTarget;
    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IMUparameters.loggingEnabled = true;
        IMUparameters.loggingTag = "IMU";
        IMUparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUparameters);

        waitForStart();
        
        mecanumEncoder(DRIVE_SPEED, 12, 8, 5.0, "vertical");

    }

    public void mecanumEncoder(double speed, double leftInches, double rightInches, double timeoutS, String direction) {
        int FLTarget = 0;
        int FRTarget = 0;
        int BLTarget = 0;
        int BRTarget = 0;
        
        /*telemetry.addData("currentposL: ", robot.motorLeft.getTargetPosition());
        telemetry.addData("currentposR: ", robot.motorRight.getTargetPosition());
        telemetry.update();*/

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            if (direction.equals("vertical")) {
                FLTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                FRTarget = robot.motorFrontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                BLTarget = robot.motorBackLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                BRTarget = robot.motorBackRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            } else if (direction.equals("lateral")) {
                FLTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                FRTarget = robot.motorFrontRight.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
                BLTarget = robot.motorBackLeft.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
                BRTarget = robot.motorBackRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            } else {
                telemetry.addLine("not a valid direction");
                telemetry.update();
            }

            double correction = checkDirection();
            robot.motorFrontLeft.setTargetPosition(FLTarget);
            robot.motorFrontRight.setTargetPosition(FRTarget);
            robot.motorBackLeft.setTargetPosition(BLTarget);
            robot.motorBackRight.setTargetPosition(BRTarget);


            // Turn On RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorFrontLeft.setPower(Math.abs(speed) - correction);
            robot.motorFrontRight.setPower(Math.abs(speed) + correction);
            robot.motorBackLeft.setPower(Math.abs(speed) - correction);
            robot.motorBackRight.setPower(Math.abs(speed) + correction);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy())) {

                //Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", FLTarget, FRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.motorFrontLeft.getCurrentPosition(),
                        robot.motorFrontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);
            telemetry.addData("Path1", "Running to %7d :%7d", FLTarget, FRTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.motorFrontLeft.getCurrentPosition(),
                    robot.motorFrontRight.getCurrentPosition());
            telemetry.update();
            /*telemetry.addData("FinalposL: ", robot.motorLeft.getTargetPosition());
            telemetry.addData("FinalposR: ", robot.motorRight.getTargetPosition());
            telemetry.update();*/

            // Turn off RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }

    public void trueDualEncoderDrive(double direction, double inches) {

    }

    public void normalDrive(double lpower, double rpower) {

        if (opModeIsActive()) {
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorFrontRight.setPower(rpower);
            robot.motorFrontLeft.setPower(lpower);

            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorBackRight.setPower(rpower);
            robot.motorBackLeft.setPower(lpower);
        }
    }

    public double pidMultiplier(double error) {
        //equation for power multiplier is x/sqrt(x^2 + C)
        int C = 100;
        return Math.abs(error / Math.sqrt((error * error) + C));
    }

    public double readAngle(String xyz) {
        Orientation angles;
        Acceleration gravity;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (xyz.equals("x")) {
            return angles.thirdAngle;
        } else if (xyz.equals("y")) {
            return angles.secondAngle;
        } else if (xyz.equals("z")) {
            return angles.firstAngle;
        } else {
            return 0;
        }
    }

    private double checkDirection() {
        double correction, angle, gain = .10;
        angle = getAngle();
        if (angle == 0)
            correction = 0;
        else
            correction = -angle;
        correction = correction * gain;
        return correction;
    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void mecanumTurn(double speed, double degrees, double timeoutS) {
        degrees = degrees / 90 * 10.5; //this was inches before. I made it so that you input degrees, then the program converts it to inches.
        int FLTarget = 0;//9.5
        int FRTarget = 0;
        int BLTarget = 0;
        int BRTarget = 0;
        /*telemetry.addData("currentposL: ", robot.motorLeft.getTargetPosition());
        telemetry.addData("currentposR: ", robot.motorRight.getTargetPosition());
        telemetry.update();*/

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            FLTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
            FRTarget = robot.motorFrontRight.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);
            BLTarget = robot.motorBackLeft.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
            BRTarget = robot.motorBackRight.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);

            double correction = checkDirection();
            robot.motorFrontLeft.setTargetPosition(FLTarget);
            robot.motorFrontRight.setTargetPosition(FRTarget);
            robot.motorBackLeft.setTargetPosition(BLTarget);
            robot.motorBackRight.setTargetPosition(BRTarget);


            // Turn On RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorFrontLeft.setPower(Math.abs(speed) - correction);
            robot.motorFrontRight.setPower(Math.abs(speed) + correction);
            robot.motorBackLeft.setPower(Math.abs(speed) - correction);
            robot.motorBackRight.setPower(Math.abs(speed) + correction);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy())) {

                //Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", FLTarget, FRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.motorFrontLeft.getCurrentPosition(),
                        robot.motorFrontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);
            telemetry.addData("Path1", "Running to %7d :%7d", FLTarget, FRTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.motorFrontLeft.getCurrentPosition(),
                    robot.motorFrontRight.getCurrentPosition());
            telemetry.update();
            /*telemetry.addData("FinalposL: ", robot.motorLeft.getTargetPosition());
            telemetry.addData("FinalposR: ", robot.motorRight.getTargetPosition());
            telemetry.update();*/

            // Turn off RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }

    public void gyroDrive(double target, String xyz, double topPower, double timeoutS) {
        //Write code to correct to a target position (NOT FINISHED)

        runtime.reset();

        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double error;
        double powerScaled = topPower;
        do {
            angle = readAngle(xyz);
            error = angle - target;
            powerScaled = topPower * (error / 180) * pidMultiplier(error);


            //double powerScaled = power*pidMultiplier(error);
            telemetry.addData("original angle", originalAngle);
            telemetry.addData("current angle", readAngle(xyz));
            telemetry.addData("error", error);
            telemetry.update();
            if (error > 0) {
                if (xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
                if (xyz.equals("x")) {
                    if (opModeIsActive()) {
                        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorFrontLeft.setPower(powerScaled);
                        robot.motorFrontRight.setPower(-powerScaled);

                        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorBackLeft.setPower(powerScaled);
                        robot.motorBackRight.setPower(-powerScaled);
                    }
                }
            } else if (error < 0) {
                if (xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
                if (xyz.equals("x")) {
                    if (opModeIsActive()) {
                        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorFrontLeft.setPower(powerScaled);
                        robot.motorFrontRight.setPower(-powerScaled);

                        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorBackLeft.setPower(powerScaled);
                        robot.motorBackRight.setPower(-powerScaled);
                    }
                }
            }
//(Math.abs(0-error)>.3)
            //(error > 0.3 && error > 0) || (error < -0.3 && error < 0)
        } while (opModeIsActive() && ((error > 0.5) || (error < -0.5)) && (runtime.seconds() < timeoutS));
        normalDrive(0, 0);

    }

}

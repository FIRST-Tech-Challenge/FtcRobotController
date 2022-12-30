package org.firstinspires.ftc.robotcontroller.external.samples;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Auto all together")
public class IMUAndPIDAndEncoder extends BasicOpMode_Linear{
    // Declare hardware:

    private ElapsedTime timer = new ElapsedTime();
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    private BNO055IMU imu         = null;

    private double  robotHeading  = 0;
    private double  headingOffset = 0;
    private double  headingError  = 0;
    private double  targetHeading = 0;

    // Drive motor position variables:
    private int lfPos;
    private int rfPos;
    private int lrPos;
    private int rrPos;

    // Set constants:
    private double fast = 0.5; // Fast speed
    private double medium = 0.3; // Medium speed
    private double slow = 0.1; // Slow speed
    private double clicksPerInch = 200; // empirically measured
    private double clicksPerDeg = 21.94; // empirically measured


    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;
    private double lastError = 0;


    Orientation angels;

    float targetAngle;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Configure hardware map:
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");

        // Set motor directions:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders:
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Let driver know encoders are reset:
        telemetry.addData("Starting at", "%7d :%7d :%7d :%7d", leftFrontMotor.getCurrentPosition(),
                rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                rightBackMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start:
        waitForStart();

        angels = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        targetAngle = angels.firstAngle;

        if(angels.firstAngle < targetAngle || angels.firstAngle > targetAngle){
            leftFrontMotor.setPower(PIDControl(targetAngle, imu.getAngularOrientation().firstAngle));
            rightFrontMotor.setPower(PIDControl(targetAngle, imu.getAngularOrientation().firstAngle));
            leftBackMotor.setPower(PIDControl(targetAngle, imu.getAngularOrientation().firstAngle));
            rightBackMotor.setPower(PIDControl(targetAngle, imu.getAngularOrientation().firstAngle));
        }

        // *****************Dead reckoning list*************
        // Distances in inches, angles in deg, speed 0.0 to 0.6
        moveForward(5, fast);

        while (opModeIsActive()){
            double lfpower = PIDControl(100,leftFrontMotor .getCurrentPosition());
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftFrontMotor.setPower(lfpower);

            double lbpower = PIDControl(100,leftBackMotor .getCurrentPosition());
            leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftBackMotor.setPower(lbpower);

            double rfpower = PIDControl(100,rightBackMotor.getCurrentPosition());
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rightFrontMotor.setPower(rfpower);

            double rbpower = PIDControl(100,rightBackMotor .getCurrentPosition());
            rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rightBackMotor.setPower(rbpower);
        }


    }


    private void moveForward(int howMuch, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch motor positions:
        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftBackMotor.getCurrentPosition();
        rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos = (int) (howMuch * clicksPerInch);
        rfPos = (int) (howMuch * clicksPerInch);
        lrPos = (int) (howMuch * clicksPerInch);
        rrPos = (int) (howMuch * clicksPerInch);

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);

        // Set the drive motor run modes to prepare for move to encoder:
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d :%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d :%7d :%7d", leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                    rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    private void moveRight(int howMuch, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch motor positions:
        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftBackMotor.getCurrentPosition();
        rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += howMuch * clicksPerInch;
        rfPos -= howMuch * clicksPerInch;
        lrPos -= howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);
        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Strafe Right");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                    rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

    }

    private void turnClockwise(int whatAngle, double speed) {
        // "whatAngle" is in degrees. A negative whatAngle turns counterclockwise.

        // Fetch motor positions:
        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftBackMotor.getCurrentPosition();
        rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += whatAngle * clicksPerDeg;
        rfPos -= whatAngle * clicksPerDeg;
        lrPos += whatAngle * clicksPerDeg;
        rrPos -= whatAngle * clicksPerDeg;

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);
        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Turn Clockwise");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                    rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
    private void moveToLine(int howMuch, double speed) {
        // "howMuch" is in inches. The robot will stop if the line is found before
        // this distance is reached. A negative howMuch moves left, positive moves right.

        // Fetch motor positions:
        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftBackMotor.getCurrentPosition();
        rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += howMuch * clicksPerInch;
        rfPos -= howMuch * clicksPerInch;
        lrPos -= howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);
        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Move To Line");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                    rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

    }
//    private float Angle () {
//
//        targetAngle = angels.firstAngle;
//        float CurrentAngle;
//        Orientation angels2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//        CurrentAngle = angels2.firstAngle;
//
//        float deltaAngle = targetAngle - CurrentAngle;
//
//        return deltaAngle;
//    }
public double PIDControl(double reference,double state){
    double error = angleWrap (reference - state);
    integralSum += error * timer.seconds();
    double derivative  = (error - lastError) / timer.seconds();
    lastError = error;

    timer.reset();

    double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);

    return output;
    }
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }
}



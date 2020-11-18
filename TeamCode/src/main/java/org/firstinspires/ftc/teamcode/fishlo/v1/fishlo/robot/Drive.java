package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Drive extends SubSystem {

    private DcMotor frontLeft, backLeft, frontRight, backRight;

    int cpr = 28;
    int gearRatio = 19;
    double diameter = 3.780;
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);
    double bias = 0.955;
    double strafeBias = 0.9;
    double conversion = cpi * bias;

    boolean exit = false;
    
    public Drive(Robot robot) {

        super(robot);
    }

    @Override
    public void init() {
        frontLeft = robot.hardwareMap.dcMotor.get("frontLeft");
        frontRight = robot.hardwareMap.dcMotor.get("frontRight");
        backLeft = robot.hardwareMap.dcMotor.get("backLeft");
        backRight = robot.hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        drive(0, 0);
    }

    boolean reverse = false;

    @Override
    public void handle() {
        double driveSpeed = -robot.gamepad1.left_stick_y;
        double turnSpeed = 0;
        double strafeSpeed = robot.gamepad1.right_stick_x;

        if (Math.abs(robot.gamepad1.left_stick_x) > 0.1) {
            turnSpeed = robot.gamepad1.left_stick_x;
        }

        if(robot.gamepad1.a) {
            reverse = false;
        }
        else if(robot.gamepad1.y) {
            reverse = true;
        }

        drive(driveSpeed, driveSpeed);
        strafe(strafeSpeed);
        drive(turnSpeed, -turnSpeed);


        robot.telemetry.addData("Drive - Dat - Drive Speed", driveSpeed);
        robot.telemetry.addData("Drive - Dat - Turn Speed", turnSpeed);
        robot.telemetry.addData("GamepadX", robot.gamepad1.left_stick_x);
        robot.telemetry.addData("Drive - Dat - Strafe Speed", strafeSpeed);
        robot.telemetry.addData("Drive - Set - frontLeft", frontLeft.getPower());
        robot.telemetry.addData("Drive - Set - backLeft", backLeft.getPower());
        robot.telemetry.addData("Drive - Set - frontRight", frontRight.getPower());
        robot.telemetry.addData("Drive - Set - backRight", backRight.getPower());
        robot.telemetry.addData("Drive - Enc - Left", frontLeft.getCurrentPosition());
        robot.telemetry.addData("Drive - Enc - Right", frontRight.getCurrentPosition());
        robot.telemetry.update();
    }

    private void left(double power) {
        try {
            frontLeft.setPower(power);
            backLeft.setPower(power);
        } catch(Exception ex) {}
    }

    private void right(double power) {
        try {
            frontRight.setPower(power);
            backRight.setPower(power);
        } catch(Exception ex) {}
    }

    public void drive(double leftPower, double rightPower) {
        left(leftPower);
        right(rightPower);
    }

    public void strafe(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(power);
    }

    public void turn (double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }

    public void moveToPosition(double inches, double speed) {
        int move = (int)(Math.round(inches*conversion));

        encoderReset();

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(speed, speed);

        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
            if (exit) {
                return;
            }
        }
        stop();
        encoderReset();
        return;
    }


    public void strafeToPosition(double inches, double speed) {
        int move = (int)(Math.round(inches * cpi * strafeBias));

        encoderReset();

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(speed, speed);

        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) {}
        stop();
        encoderReset();
        return;
    }

    public void stop() {
        drive(0, 0);
    }

/*
    int base = 0;
    public void resetEncoder() {
        base = backRight.getCurrentPosition();
    }
*/

    public void encoderReset() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

/*
    public int getEncoder() {
        return backRight.getCurrentPosition() - base;
    }
*/



}

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

    LinearOpMode opmode;

    int cpr = 28;
    int gearRatio = 19;
    double diameter = 3.780;
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);
    double bias = 0.8;
    double strafeBias = 0.9;

    double conversion = cpi * bias;
    boolean exit = false;

    double yaw = 0;
    double gyroOffset = 0;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    public Drive(Robot robot, LinearOpMode opmode) {
        super(robot);
        this.opmode = opmode;
    }

    @Override
    public void init() {
        frontLeft = robot.hardwareMap.dcMotor.get("front_left");
        frontRight = robot.hardwareMap.dcMotor.get("front_right");
        backLeft = robot.hardwareMap.dcMotor.get("back_Left");
        backRight = robot.hardwareMap.dcMotor.get("back_right");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        drive(0, 0);
    }

    boolean reverse = false;

    @Override
    public void handle() {
        double speed = robot.gamepad1.right_trigger - robot.gamepad1.left_trigger;
        double direction = -robot.gamepad1.left_stick_x;

        if(robot.gamepad1.a) {
            reverse = false;
        } else if(robot.gamepad1.y) {
            reverse = true;
        }

        DriveDir driveDir = getDir(direction);

        if(Math.abs(speed) < 5 && driveDir != DriveDir.STRAIGHT) {
            drive(-direction, direction);
        } else {
            if (reverse) {
                drive(driveDir.getLeft(reverse) * speed, driveDir.getRight(reverse) * speed);
            } else {
                drive(driveDir.getRight(reverse) * speed, driveDir.getLeft(reverse) * speed);
            }
        }

        robot.telemetry.addData("Drive - Dat - speed", speed);
        robot.telemetry.addData("Drive - Dat - direction", driveDir.name());
        robot.telemetry.addData("Drive - Set - frontLeft", frontLeft.getPower());
        robot.telemetry.addData("Drive - Set - backLeft", backLeft.getPower());
        robot.telemetry.addData("Drive - Set - frontRight", frontRight.getPower());
        robot.telemetry.addData("Drive - Set - backRight", backRight.getPower());
        robot.telemetry.addData("Drive - Enc - Left", frontLeft.getCurrentPosition());
        robot.telemetry.addData("Drive - Enc - Right", frontRight.getCurrentPosition());
    }

    private DriveDir getDir(double dir) {
        if(dir <= -0.7) {
            return DriveDir.HARD_LEFT;
        } else if(dir <= -0.4) {
            return DriveDir.MEDIUM_LEFT;
        } else if(dir <= -0.1) {
            return DriveDir.EASY_LEFT;
        } else if(dir >= 0.7) {
            return DriveDir.HARD_RIGHT;
        } else if(dir >= 0.4) {
            return DriveDir.MEDIUM_RIGHT;
        } else if(dir >= 0.1) {
            return DriveDir.EASY_RIGHT;
        } else {
            return DriveDir.STRAIGHT;
        }
    }

    public enum DriveDir {
        HARD_LEFT(-1, 1),
        MEDIUM_LEFT(0, 1),
        EASY_LEFT(0.5, 1),
        STRAIGHT(1, 1),
        EASY_RIGHT(1, 0.5),
        MEDIUM_RIGHT(1, 0),
        HARD_RIGHT(1, -1);

        private double left, right;

        DriveDir(double left, double right) {
            this.left = left;
            this.right = right;
        }

        public double getLeft(boolean reverse) {
            return reverse ? -left : left;
        }

        public double getRight(boolean reverse) {
            return reverse ? -right : right;
        }
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

    public void drive(double left, double right) {
        left(left);
        right(right);
    }

    public void moveToPosition(double inches, double speed) {
        int move = (int)(Math.round(inches*conversion));

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(speed, speed);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            if (exit) {
                stop();
                return;
            }
        }
        stop();
        return;
    }

    public void strafeToPosition(double inches, double speed) {
        int move = (int)(Math.round(inches * cpi * strafeBias));

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(speed, speed);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {}
        stop();
        return;
    }

    public void stop() {
        drive(0, 0);
    }


    int base = 0;
    public void resetEncoder() {
        base = backRight.getCurrentPosition();
    }

    public int getEncoder() {
        return backRight.getCurrentPosition() - base;
    }

    public void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = robot.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void turnWithGyro (double degrees, double speedDir) {
        getHeading();
        resetGyro();

        double first;
        double second;

        if (speedDir > 0) {
            if (degrees > 10) {
                first = (degrees - 10) + devert(yaw);
                second = degrees + devert(yaw);
            }
            else {
                first = devert(yaw);
                second = degrees + devert(yaw);
            }
        }
        else {
            if (degrees > 10) {
                first = devert(-(degrees - 10) + devert(yaw));
                second = devert(-degrees + devert(yaw));
            }
            else {
                first = devert(yaw);
                second = devert(-degrees + devert(yaw));
            }
        }

        double firsta = convert(first - 5);
        double firstb = convert(first + 5);

        turnWithEncoder(speedDir);

        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opmode.opModeIsActive()) {
                getHeading();
                gravity = imu.getGravity();
                robot.telemetry.addData("Position", yaw);
                robot.telemetry.update();
            }
        }
        else {
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opmode.opModeIsActive()) {
                getHeading();
                gravity = imu.getGravity();
                robot.telemetry.addData("Position", yaw);
                robot.telemetry.update();
            }
        }

        double seconda = convert(second - 5);
        double secondb = convert(second + 5);

        turnWithEncoder(speedDir / 3);

        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opmode.opModeIsActive()) {
                getHeading();
                gravity = imu.getGravity();
                robot.telemetry.addData("Position", yaw);
                robot.telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) &&  opmode.opModeIsActive()) {
                getHeading();
                gravity = imu.getGravity();
                robot.telemetry.addData("Position", yaw);
                robot.telemetry.update();
            }
            stop();
        }
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double devert (double degrees) {
        if (degrees < 0) {
            degrees += 360;
        }
        return degrees;
    }

    public double convert (double degrees) {
        if (degrees > 179) {
            degrees = -(360-degrees);
        }
        else if (degrees < -180) {
            degrees = 360 + degrees;
        }
        else if (degrees > 360) {
            degrees = degrees - 360;
        }
        return degrees;
    }

    public void turnWithEncoder (double input) {
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left(input);
        right(-input);
    }

    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        yaw = -angles.firstAngle;

        return (convert(yaw - gyroOffset));
    }

    public void resetGyro() {
        gyroOffset = yaw;
    }
}

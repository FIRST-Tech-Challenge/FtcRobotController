package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


//I Think this should work. I still need to test though.
public class Caleb_drive extends LinearOpMode {
    private DcMotor frontLeft, backLeft, frontRight, backRight;

    public Caleb_drive() {
        super();
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    // @Override
    public void init7() {
        frontLeft = hardwareMap.dcMotor.get("front_left");
        backLeft = hardwareMap.dcMotor.get("back_left");
        frontRight = hardwareMap.dcMotor.get("front_right");
        backRight = hardwareMap.dcMotor.get("back_right");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        drive(0, 0);
    }

    boolean reverse = false;

    //@Override
    public void handle() {
        double speed = gamepad1.right_trigger - gamepad1.left_trigger;
        double direction = gamepad1.left_stick_x;

        if(gamepad1.x) {
            reverse = false;
        } else if(gamepad1.y) {
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

        telemetry.addData("Drive - Dat - speed", speed);
        telemetry.addData("Drive - Dat - direction", driveDir.name());
        telemetry.addData("Drive - Set - frontLeft", frontLeft.getPower());
        telemetry.addData("Drive - Set - backLeft", backLeft.getPower());
        telemetry.addData("Drive - Set - frontRight", frontRight.getPower());
        telemetry.addData("Drive - Set - backRight", backRight.getPower());
        telemetry.addData("Drive - Enc - Left", frontLeft.getCurrentPosition());
        telemetry.addData("Drive - Enc - Right", frontRight.getCurrentPosition());
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

    public void drive(double speed, DriveDir driveDir) {
        if(reverse) {
            drive(driveDir.getLeft(reverse) * speed, driveDir.getRight(reverse) * speed);
        } else {
            drive(driveDir.getRight(reverse) * speed, driveDir.getLeft(reverse) * speed);
        }
    }

    public void stop7 () {
        drive(0, 0);
    }


    int base = 0;
    public void resetEncoder() {
        base = backRight.getCurrentPosition();
    }

    public int getEncoder() {
        return backRight.getCurrentPosition() - base;
    }
}

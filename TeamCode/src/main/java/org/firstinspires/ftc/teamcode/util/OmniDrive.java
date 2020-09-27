package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class OmniDrive {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public enum Direction {
        FORWARD,
        LEFT,
        RIGHT,
        BACKWARD,
        FORWARD_LEFT,
        FORWARD_RIGHT,
        BACKWARD_LEFT,
        BACKWARD_RIGHT,
        ROTATE_LEFT,
        ROTATE_RIGHT
    }

    public OmniDrive(DcMotor frontLeft, DcMotor frontRight,
                     DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

    }

    public void moveForward(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void moveBackward(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
    }

    public void moveLeft(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    public void moveRight(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }

    public void rotateRight(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    public void rotateLeft(double power) {
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(-power);
            backRight.setPower(power);
    }

    public void moveForwardLeft(double power) {
        frontLeft.setPower(0);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(0);
    }

    public void moveForwardRight(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(power);
    }

    public void moveBackwardLeft(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(-power);
    }

    public void moveBackwardRight(double power) {
        frontLeft.setPower(0);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(0);
    }

    public void circleMove(double x, double y) {
        double power = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        double degrees = Math.toDegrees(Math.atan(Math.abs(y)/Math.abs(x)));
        double directionPower = (degrees - 45) / 45;
        //double deadzone = 0;

        if (power == 0) {
            stopDrive();
        } else {
            if (x == 0) {
                moveForward(-y);
            } else if (y == 0) {
                if (x > 0) {
                    moveRight(x);
                } else {
                    moveLeft(x);
                }
            } else {
                if (x > 0 && -y > 0) {
                    // QUAD I
                    frontLeft.setPower(power);
                    frontRight.setPower(directionPower);
                    backLeft.setPower(directionPower);
                    backRight.setPower(power);
                } else if (x < 0 && -y > 0) {
                    // QUAD 2
                    frontLeft.setPower(directionPower);
                    frontRight.setPower(power);
                    backLeft.setPower(power);
                    backRight.setPower(directionPower);
                } else if (x < 0 && -y < 0) {
                    // QUAD 3
                    frontLeft.setPower(-power);
                    frontRight.setPower(-directionPower);
                    backLeft.setPower(-directionPower);
                    backRight.setPower(-power);

                } else {
                    // QUAD 4
                    frontLeft.setPower(power);
                    frontRight.setPower(directionPower);
                    backLeft.setPower(directionPower);
                    backRight.setPower(power);
                }

            }
        }
    }

    public void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void setMode(DcMotor.RunMode runMode) {
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    public void dpadMove(Gamepad gamepad, float power, boolean reverse) {
        if (reverse) {
            if (gamepad.dpad_up && gamepad.dpad_left) {
                moveBackwardRight(power);
            } else if (gamepad.dpad_up && gamepad.dpad_right) {
                moveBackwardLeft(power);
            } else if (gamepad.dpad_down && gamepad.dpad_left) {
                moveForwardRight(power);
            } else if (gamepad.dpad_down && gamepad.dpad_right) {
                moveForwardLeft(power);
            } else if (gamepad.dpad_up) {
                moveBackward(power);
            } else if (gamepad.dpad_left) {
                moveRight(power);
            } else if (gamepad.dpad_right) {
                moveLeft(power);
            } else if (gamepad.dpad_down) {
                moveForward(power);
            } else {
                stopDrive();
            }
        } else {
            if (gamepad.dpad_up && gamepad.dpad_left) {
                moveForwardLeft(power);
            } else if (gamepad.dpad_up && gamepad.dpad_right) {
                moveForwardRight(power);
            } else if (gamepad.dpad_down && gamepad.dpad_left) {
                moveBackwardLeft(power);
            } else if (gamepad.dpad_down && gamepad.dpad_right) {
                moveBackwardRight(power);
            } else if (gamepad.dpad_up) {
                moveForward(power);
            } else if (gamepad.dpad_left) {
                moveLeft(power);
            } else if (gamepad.dpad_right) {
                moveRight(power);
            } else if (gamepad.dpad_down) {
                moveBackward(power);
            } else {
                stopDrive();
            }
        }

        if (gamepad.left_trigger > 0) {
            rotateLeft(gamepad.left_trigger);
        } else if (gamepad.right_trigger > 0) {
            rotateRight(gamepad.right_trigger);
        }

        if (gamepad.left_bumper) {
            rotateLeft(0.25f);
        } else if (gamepad.right_bumper) {
            rotateRight(0.25f);
        }
    }
}

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

    /**
     * Move and/or rotate the robot along an axis relative to the robot's center.

     * @param power The power to move everything
     * @param angle The heading to move the robot in, 0 is forward rotating clockwise.
     * @param rotation How fast to rotate from -1 to 1 where 1 indicates CW rotation and full rotation w/ no movement.
     */
    public void move(double power, double angle, double rotation) {
        double pi4 = Math.PI / 4;

        // Get raw powers
        double fl_power = power * Math.sin(angle + pi4) + rotation;
        double fr_power = power * Math.cos(angle + pi4) - rotation;
        double bl_power = power * Math.cos(angle + pi4) + rotation;
        double br_power = power * Math.sin(angle + pi4) - rotation;

        // Calculate unit vector and apply power magnitude
        double power_max = Math.max(Math.max(fl_power, fr_power), Math.max(bl_power, br_power));

        // Scale power if one motor exceeds 1
        if (power_max > 1.0) {
            fl_power = fl_power / power_max;
            fr_power = fr_power / power_max;
            bl_power = bl_power / power_max;
            br_power = br_power / power_max;
        }
        // Set motor powers
        frontLeft.setPower(fl_power);
        frontRight.setPower(fr_power);
        backLeft.setPower(bl_power);
        backRight.setPower(br_power);
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

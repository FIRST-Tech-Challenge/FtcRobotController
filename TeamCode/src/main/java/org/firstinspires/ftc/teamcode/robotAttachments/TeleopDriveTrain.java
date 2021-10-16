package org.firstinspires.ftc.teamcode.robotAttachments;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleopDriveTrain {
    private DcMotor back_right;
    private DcMotor back_left;
    private DcMotor front_right;
    private DcMotor front_left;
    private double DrivePowerMult;

    public TeleopDriveTrain(HardwareMap hardwareMap, String frontRight, String frontLeft, String backRight, String backLeft) {
        /*
        front_right = hardwareMap.dcMotor.get("back_left");
        front_left = hardwareMap.dcMotor.get("back_right");
        back_right = hardwareMap.dcMotor.get("front_left");
        back_left = hardwareMap.dcMotor.get("front_right");
         */
        front_right = hardwareMap.dcMotor.get(frontRight);
        front_left = hardwareMap.dcMotor.get(frontLeft);
        back_right = hardwareMap.dcMotor.get(backRight);
        back_left = hardwareMap.dcMotor.get(backLeft);

        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);

        this.DrivePowerMult = 1;

    }

    public void setPowerFromGamepad(Gamepad gamepad) {
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottom most position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        back_left.setPower(DrivePowerMult * ((gamepad.left_stick_y + gamepad.left_stick_x) + gamepad.right_stick_x));
        front_left.setPower(DrivePowerMult * ((gamepad.left_stick_y - gamepad.left_stick_x) + gamepad.right_stick_x));
        back_right.setPower(DrivePowerMult * ((gamepad.left_stick_y - gamepad.left_stick_x) - gamepad.right_stick_x));
        front_right.setPower(DrivePowerMult * ((gamepad.left_stick_y + gamepad.left_stick_x) - gamepad.right_stick_x));
    }

    public void setDrivePowerMult(double drivePowerMult) {
        this.DrivePowerMult = drivePowerMult;
    }

    public void stopAll() {
        back_right.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        front_left.setPower(0);
    }

    public void strafeAtAngle(double angle, double power) {
        power = boundNumber(power);
        double power1;
        double power2;

        angle = angle % 360;

        power1 = -Math.cos(Math.toRadians(angle + 45.0));
        power2 = -Math.cos(Math.toRadians(angle - 45));

        power1 = power * power1;
        power2 = power * power2;

        front_right.setPower(power1);
        back_left.setPower(power1);

        front_left.setPower(power2);
        back_right.setPower(power2);

    }

    public void reInitMotors() {
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private static double boundNumber(double num) {
        if (num > 1) {
            num = 1;
        }
        if (num < -1) {
            num = -1;
        }
        return num;
    }
}

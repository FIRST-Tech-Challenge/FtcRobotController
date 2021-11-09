package org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleopDriveTrain {

    private final BasicDrivetrain drivetrain;
    private double DrivePowerMult;

    public TeleopDriveTrain(HardwareMap hardwareMap, String frontRight, String frontLeft, String backRight, String backLeft) {
        drivetrain = new BasicDrivetrain(hardwareMap, frontRight, frontLeft, backRight, backLeft);

        this.DrivePowerMult = 1;

    }


    public void setPowerFromGamepad(Gamepad gamepad) {

        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottom most position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        drivetrain.back_left.setPower(DrivePowerMult * ((gamepad.left_stick_y + gamepad.left_stick_x) - gamepad.right_stick_x));
        drivetrain.front_left.setPower(DrivePowerMult * ((gamepad.left_stick_y - gamepad.left_stick_x) - gamepad.right_stick_x));
        drivetrain.back_right.setPower(DrivePowerMult * ((gamepad.left_stick_y - gamepad.left_stick_x) + gamepad.right_stick_x));
        drivetrain.front_right.setPower(DrivePowerMult * ((gamepad.left_stick_y + gamepad.left_stick_x) + gamepad.right_stick_x));


    }

    public void setDrivePowerMult(double drivePowerMult) {
        this.DrivePowerMult = drivePowerMult;
    }

    public void stopAll() {
        drivetrain.stopAll();
    }

    public void strafeAtAngle(double angle, double power) {
        drivetrain.strafeAtAngle(angle, power);
    }

    public void reInitMotors() {
        drivetrain.reinitializeMotors();
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

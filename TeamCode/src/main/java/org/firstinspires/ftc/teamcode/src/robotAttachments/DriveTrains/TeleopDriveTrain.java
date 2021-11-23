package org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleopDriveTrain extends BasicDrivetrain {
    private double DrivePowerMult;

    public TeleopDriveTrain(HardwareMap hardwareMap, String frontRight, String frontLeft, String backRight, String backLeft) {
        super(hardwareMap, frontRight, frontLeft, backRight, backLeft);
        this.DrivePowerMult = 1;
    }

    public void setPowerFromGamepad(Gamepad gamepad) {

        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottom most position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        this.back_left.setPower(DrivePowerMult * ((gamepad.left_stick_y + gamepad.left_stick_x) - gamepad.right_stick_x));
        this.front_left.setPower(DrivePowerMult * ((gamepad.left_stick_y - gamepad.left_stick_x) - gamepad.right_stick_x));
        this.back_right.setPower(DrivePowerMult * ((gamepad.left_stick_y - gamepad.left_stick_x) + gamepad.right_stick_x));
        this.front_right.setPower(DrivePowerMult * ((gamepad.left_stick_y + gamepad.left_stick_x) + gamepad.right_stick_x));


    }

    public void setDrivePowerMult(double drivePowerMult) {
        this.DrivePowerMult = drivePowerMult;
    }

}

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.settings.HardwareMapDeviceNames;

import java.util.ArrayList;

@TeleOp (name="Josh: 4 Wheel AWD Test", group="Josh")
public class Drive extends OpMode {
    ArrayList<DcMotor> leftDriveMotors;
    ArrayList<DcMotor>  rightDriveMotors;

    @Override
    public void init() {
        // Left Drive
        leftDriveMotors = new ArrayList<DcMotor>();
        for (String name : HardwareMapDeviceNames.LEFT_DRIVE) {
            leftDriveMotors.add(hardwareMap.get(DcMotor.class, name));
        }

        // Right Drive
        rightDriveMotors = new ArrayList<DcMotor>();
        for (String name : HardwareMapDeviceNames.RIGHT_DRIVE) {
            rightDriveMotors.add(hardwareMap.get(DcMotor.class, name));
        }
    }

    @Override
    public void loop() {
        double lPower = gamepad1.left_stick_y;
        double rPower = gamepad1.right_stick_y;
        for (DcMotor leftMotor : leftDriveMotors) {
            leftMotor.setPower(lPower);
        }

        for (DcMotor rightMotor : rightDriveMotors) {
            rightMotor.setPower(rPower);
        }

        telemetry.addData("Motors ", lPower + "l | " + rPower + "r");
    }
}

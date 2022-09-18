package org.firstinspires.ftc.forteaching.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.forteaching.TankDriveDemo;

@Disabled
@TeleOp(name = "StickTankDriveForVideo")
public class StickTankDriveForVideo extends OpMode {

    private static final double DEAD_ZONE = 0.1;
    private TankDriveDemo tankDrive;
    private DcMotorEx motorL;
    private DcMotorEx motorR;

    @Override
    public void init() {
        motorL = hardwareMap.get(DcMotorEx.class, "motorL");
        motorR = hardwareMap.get(DcMotorEx.class, "motorR");
        motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        tankDrive = new TankDriveDemo(motorL, motorR);
    }

    @Override
    public void loop() {
        if (Math.abs(gamepad1.left_stick_y) > DEAD_ZONE) {
            tankDrive.motorLPower(gamepad1.left_stick_y);
        } else {
            tankDrive.motorLPower(0);
        }
        if (Math.abs(gamepad1.right_stick_y) > DEAD_ZONE) {
            tankDrive.motorRPower(gamepad1.right_stick_y);
        } else {
            tankDrive.motorRPower(0);
        }
    }

    @Override
    public void start() {

    }
}

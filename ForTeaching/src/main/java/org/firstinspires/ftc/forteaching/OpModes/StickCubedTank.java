package org.firstinspires.ftc.forteaching.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.forteaching.TankDriveDemo;

@TeleOp(name = "TankCubed")
public class StickCubedTank extends OpMode {
    
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
        float ly = gamepad1.left_stick_y;
        float ry = gamepad1.right_stick_y;
        if (Math.abs(ly) > DEAD_ZONE) {
            tankDrive.motorLPower(ly * ly * ly);
        } else {
            tankDrive.motorLPower(0);
        }
        if (Math.abs(ry) > DEAD_ZONE) {
            tankDrive.motorRPower(ry * ry * ry);
        } else {
            tankDrive.motorRPower(0);
        }
    }

    @Override
    public void start() {

    }
}

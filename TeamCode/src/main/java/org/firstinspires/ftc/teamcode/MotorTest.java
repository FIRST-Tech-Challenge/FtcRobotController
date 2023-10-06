package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorTest extends LinearOpMode {
    private DcMotor TestMotor = null;
    @Override
    public void runOpMode() {
        TestMotor = hardwareMap.get(DcMotor.class, "motor1");
        TestMotor.setPower(0.75);
    }
}

package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ViperTest extends OpMode {

    private DcMotor motor;

    public void init(){
        motor = hardwareMap.dcMotor.get("motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void loop() {

        if (gamepad1.right_stick_y != 0.0) {
            motor.setPower(0.5*gamepad1.right_stick_y);
        }
        else{
            motor.setPower(0);

        }

    }

}



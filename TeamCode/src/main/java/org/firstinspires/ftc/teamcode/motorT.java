package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="motorProgram")

public class motorT extends OpMode {
    DcMotor motor;
    @Override
    public void init(){
        motor = hardwareMap.get(DcMotor.class, "motorTest");
    }
    @Override
    public void loop(){


        motor.setPower(1);
        //delay 500;
        //motor.setpower(-1):
        //delay
    }
}

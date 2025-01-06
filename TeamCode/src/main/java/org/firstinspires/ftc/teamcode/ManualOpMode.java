package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ManualOpMode extends OpMode {

    Moving moving;
    Claw claw;


    @Override
    public void init(){

        moving = new Moving();
        claw = new Claw();
        moving.setHW(hardwareMap, telemetry, gamepad1);
    }
    public void loop (){
        moving.move();
    }
}

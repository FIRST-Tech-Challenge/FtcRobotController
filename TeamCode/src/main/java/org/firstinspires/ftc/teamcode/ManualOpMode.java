package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ManualOpMode extends OpMode {

    Moving moving;

    @Override
    public void init(){

        moving = new Moving();
        moving.setHW(hardwareMap, telemetry, gamepad1);
    }
    public void loop (){
        moving.move();
    }
}

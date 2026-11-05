package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class indexOpMode extends OpMode {

    index index = new index();

    @Override
    public void init() {
        index.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            index.setMotorSpeed(1);
        }
        else{
            index.setMotorSpeed(0);
        }
    }
}

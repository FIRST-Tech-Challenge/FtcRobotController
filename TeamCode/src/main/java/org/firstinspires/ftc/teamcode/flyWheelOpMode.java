package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class flyWheelOpMode extends OpMode {

    flyWheelOne one = new flyWheelOne();
    flyWheelTwo two = new flyWheelTwo();


    @Override
    public void init() {
        one.init(hardwareMap);
        two.init(hardwareMap);

    }

    @Override
    public void loop() {
        if (gamepad1.a){
            one.setMotorSpeed(1);
            two.setMotorSpeed(1);
        }
        else{
            one.setMotorSpeed(0);
            two.setMotorSpeed(0);
        }
    }
}

package org.firstinspires.ftc.teamcode.robots.tempbot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Temp_6832", group = "Challenge")
@Config
public class Temp_6832 extends OpMode {

    static UnderArm underArm;

    public void init(){
        underArm = new UnderArm(hardwareMap,false);
    }

    public void init_loop(){
        underArm.update();
    }
    public void loop(){

        if(Math.abs(gamepad1.left_stick_y) > 0.05){
            underArm.adjustShoulder(gamepad1.left_stick_y);
        }
        if(Math.abs(gamepad1.right_stick_y) > 0.05){
            underArm.adjustElbow(gamepad1.right_stick_y);
        }
        if(Math.abs(gamepad1.right_stick_x) > 0.05){
            underArm.adjustLasso(gamepad1.right_stick_x);
        }
        if(Math.abs(gamepad1.left_stick_x) > 0.05){
            underArm.adjustTurret(gamepad1.left_stick_x);
        }

        underArm.update();
    }

}

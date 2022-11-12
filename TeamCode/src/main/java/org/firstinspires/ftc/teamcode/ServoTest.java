package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest")
public class ServoTest extends OpMode {
    // servo declarations
    private Servo rightServo;
    private Servo leftServo;

    @Override
    public void init(){

    }

    @Override
    public void loop(){
        if(gamepad1.b){
            rightServo.setPosition(0.5);
            leftServo.setPosition(0.5);
        }

        if(gamepad1.a){
            rightServo.setPosition(0);
            leftServo.setPosition(0);
        }

    }

}

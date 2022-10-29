package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoTeleop", group="A")
public class ServoTeleop extends DriveMethods{
    Servo servoGrabber;
    double servoPosition = 0.5;
    @Override
    public void runOpMode() {
        servoGrabber = hardwareMap.get(Servo.class, "servo");
        waitForStart();


        while(opModeIsActive()){
            if(gamepad2.dpad_up){
                servoPosition = servoPosition + 0.01;
            }
            if(gamepad2.dpad_down){
                servoPosition = servoPosition - 0.01;
            }

            if(gamepad2.dpad_right){
                servoPosition = servoPosition + 0.05;
            }
            if(gamepad2.dpad_left){
                servoPosition = servoPosition - 0.05;
            }

            servoGrabber.setPosition(servoPosition);

        }
    }
}

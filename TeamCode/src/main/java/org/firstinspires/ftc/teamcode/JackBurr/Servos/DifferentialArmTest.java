package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class DifferentialArmTest extends OpMode {
    public Servo grippers;
    public Servo left_servo;
    public Servo right_servo;
    public ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        grippers = hardwareMap.get(Servo.class, "grippers");
        left_servo = hardwareMap.get(Servo.class, "left_diff");
        right_servo = hardwareMap.get(Servo.class, "right_diff");
    }

    @Override
    public void loop() {
        //-------------------------------------------------------------------------------------------------------------------------
        //SAME DIRECTION: ROTATE GRIPPERS
        //  - LEFT:
        //      - BOTH LEFT
        //OPPOSITE DIRECTION: ROTATE ARM
        //  - TOWARDS ROBOT:
        //      - LEFT GOES RIGHT
        //      - RIGHT GOES LEFT
        //------------------------------------------------------------------------------------------------------------------------
        if(gamepad1.right_trigger > 0 && timer.seconds() > 0.3){
            if(grippers.getPosition() == 0){
                grippers.setPosition(0.6);
            }
            else{
                grippers.setPosition(0);
            }
            timer.reset();
        }
        //----------------------------------------------------------------------------------------------------------
        if(gamepad1.dpad_left && timer.seconds() > 0.3){
            if(left_servo.getPosition() >= 0.1){
                left_servo.setPosition(left_servo.getPosition() - 0.1);
            }
            if(right_servo.getPosition() >= 0.1){
                right_servo.setPosition(right_servo.getPosition() - 0.1);
            }
            timer.reset();
        }
        if(gamepad1.dpad_right && timer.seconds() > 0.3){
            if(left_servo.getPosition() <= 0.9){
                left_servo.setPosition(left_servo.getPosition() + 0.1);
            }
            if(right_servo.getPosition() <= 0.9){
                right_servo.setPosition(right_servo.getPosition() + 0.1);
            }
            timer.reset();
        }

        if(gamepad1.dpad_up && timer.seconds() > 0.3){
            left_servo.setPosition(left_servo.getPosition() + 0.1);
            right_servo.setPosition(right_servo.getPosition() - 0.1);
            timer.reset();
        }
        if(gamepad1.dpad_down && timer.seconds() > 0.3){
            left_servo.setPosition(left_servo.getPosition() - 0.1);
            right_servo.setPosition(right_servo.getPosition() + 0.1);
            timer.reset();
        }

    }
}

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends OpMode {
    private Servo intakeServo;

    @Override
    public void init(){
        telemetry.addData("Initiated", "True");
    }

    @Override
    public void loop(){
        if(gamepad1.y) {
            // move to 0 degrees.
            intakeServo.setPosition(0);
        } else if (gamepad1.x || gamepad1.b) {
            // move to 90 degrees.
            intakeServo.setPosition(22/14);
        } else if (gamepad1.a) {
            // move to 180 degrees.
            intakeServo.setPosition(22/7);
        }
    }
}

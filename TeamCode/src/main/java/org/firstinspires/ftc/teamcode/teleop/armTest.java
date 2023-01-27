package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.gamepadEX.gamepadEX;

@Disabled
@TeleOp(name="Servo Arm Test (OUTDATED)", group="OUTDATED TEST")
public class armTest extends OpMode{
    private Servo wrist;
    private Servo arm;
    private boolean armState = true;
    private boolean aState = false;

    @Override
    public void init() {
        telemetry.addData("Status: ", "Initialising");
        telemetry.update();
        wrist = hardwareMap.get(Servo.class, "wrist");
        arm = hardwareMap.get(Servo.class, "arm");
        telemetry.addData("Status: ", "Initialised");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        //vision detect loop goes here
    }

    @Override
    public void start() {
        //set claw to halfway point goes here
    }

    @Override
    public void loop() {
        if(gamepad1.a && !aState){
            aState = true;
            armState = !armState;
        }
        else if(!gamepad1.a && aState){
            aState = false;
        }

        if(armState){
            arm.setPosition(0.1);
            wrist.setPosition(0.2);
        }
        else{
            arm.setPosition(0.9);
            wrist.setPosition(1.0);
        }

    }
}

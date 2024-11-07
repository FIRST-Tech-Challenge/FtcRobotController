package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.internal.network.WifiUtil;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.MecanumEncoder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

@TeleOp(name = "servo test")
public class WristTest extends OpMode {


    private Servo arm1 = null;
    private Servo arm2 = null;

    public void ProcessWrist() {
        arm1.setPosition((gamepad1.right_stick_x+1.0)/2);
        arm2.setPosition((gamepad1.right_stick_x+1.0)/2);
    }
    @Override
    public void init() {
        // run once when init is pressed
        arm1 = hardwareMap.get(Servo .class, "arm1");
        arm2 = hardwareMap.get(Servo .class, "arm2");
    }

    @Override
    public void init_loop() {
        // add stuff here for the init loop
    }

    @Override
    public void loop() {
        // runs while in play
        ProcessWrist();


    }
}

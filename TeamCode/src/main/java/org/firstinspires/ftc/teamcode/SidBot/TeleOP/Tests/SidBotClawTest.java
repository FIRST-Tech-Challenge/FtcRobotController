package org.firstinspires.ftc.teamcode.SidBot.TeleOP.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LinearSlide;
import org.firstinspires.ftc.teamcode.ServoClaw;

@TeleOp(name="SidBotClawTest ClawTest", group="tests")
public class SidBotClawTest extends OpMode {

    private ServoClaw clawObject;
    private LinearSlide armObject;

    private float slowdownModifierP1;

    @Override
    public void init() {
        //Telemetry A
        telemetry.addData("Hello! Initializing!", "＼(⌒▽⌒)");
        telemetry.update();

        //Get hardware map things
        Servo claw = hardwareMap.servo.get("claw");
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        // Init Claw Object
        this.clawObject = new ServoClaw(claw,0.33f,1);

        // Init Arm Object
        // NOTE: Arm is unrestricted
        this.armObject = new LinearSlide(arm, 0, 360);

        //Telemetry B
        telemetry.addData("Ready for launch!" , "＼(≧▽≦)／");
        telemetry.addData("WARNING!" , "ARM IS OPERATING IN UNRESTRICTED MODE");
        telemetry.update();
    }

    @Override
    public void loop() {

        //Get Slowdown Modifier
        this.slowdownModifierP1 = 1 - (gamepad1.right_trigger * 0.85f);

        //Manage claw
        clawObject.actuateToPercent(gamepad1.left_trigger);

        //Manage arm
        armObject.MoveSlideUnrestricted(gamepad1.left_stick_y * slowdownModifierP1);

        //Telemetry
        telemetry.addData("Claw Position", clawObject.getPos());
        telemetry.addData("Rotation", gamepad1.left_stick_y * slowdownModifierP1);
        telemetry.update();
    }
}

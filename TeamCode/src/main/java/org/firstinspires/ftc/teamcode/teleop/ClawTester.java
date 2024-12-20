package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
@TeleOp(name = "ClawTester")
public class ClawTester extends LinearOpMode {
    private Claw claw;
    private GamepadEvents controller;
    private double clawPos;
    public void runOpMode(){
        claw = new Claw(hardwareMap);
        controller = new GamepadEvents(gamepad1);
        telemetry.addData("Status","Initialized");
        clawPos = 0.6;
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            if(controller.b.onPress()){
                //close
                claw.toggle();
                telemetry.addData("Claw is Open", claw.getIsOpen());
            }

//            telemetry.addData("Claw Position", clawPos);
            telemetry.update();
            controller.update();
        }
    }
}

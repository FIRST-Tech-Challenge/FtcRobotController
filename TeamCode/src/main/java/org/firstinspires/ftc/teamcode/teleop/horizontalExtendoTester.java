package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.horizontalExtendo;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

public class horizontalExtendoTester extends LinearOpMode {
    private horizontalExtendo extendo;
    private GamepadEvents controller;
    @Override
    public void runOpMode() throws InterruptedException {
        extendo = new horizontalExtendo(hardwareMap,"hExtendo");

        waitForStart();
        while(opModeIsActive())
        {
            int extendoPos = (int)((controller.left_trigger.getTriggerValue() - controller.right_trigger.getTriggerValue()) * 10);
            extendo.setPosition(extendoPos);
            telemetry.addData("Extendo pos: ", extendoPos);
        }
    }
}

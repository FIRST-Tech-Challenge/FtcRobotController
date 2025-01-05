package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.horizontalExtendo;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
@TeleOp(name ="hExtendo Tester")
public class horizontalExtendoTester extends LinearOpMode {
    private horizontalExtendo extendo;
    private GamepadEvents controller;
    @Override
    public void runOpMode() throws InterruptedException {
        extendo = new horizontalExtendo(hardwareMap,"hExtendo");
        controller = new GamepadEvents(gamepad1);
        waitForStart();
        while(opModeIsActive())
        {
            double extendoPos = (controller.left_trigger.getTriggerValue() - controller.right_trigger.getTriggerValue()) * 0.001;
            extendo.setPosition(extendoPos);
            telemetry.addData("Extendo pos: ", extendo.getPosition());
            telemetry.update();
            controller.update();
        }
    }
}

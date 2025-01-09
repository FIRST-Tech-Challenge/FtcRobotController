package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.DoubleHorizontalExtendo;
import org.firstinspires.ftc.teamcode.subsystems.horizontalExtendo;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
@TeleOp(name ="hExtendo Tester")
public class HorizontalExtendoTester extends LinearOpMode {
    private DoubleHorizontalExtendo extendo;
    private GamepadEvents controller;
    @Override
    public void runOpMode() throws InterruptedException {
        extendo = new DoubleHorizontalExtendo(hardwareMap, "hExtendo", "hExtendo");
        controller = new GamepadEvents(gamepad1);
        waitForStart();
        while(opModeIsActive())
        {
            double extendoPower = controller.left_trigger.getTriggerValue() - controller.right_trigger.getTriggerValue();
            extendo.setPower(extendoPower);
            telemetry.addData("Extendo power:", extendo.getPower());
            telemetry.addData("Extendo pos: ", extendo.getPosition());
            telemetry.update();
            controller.update();
        }
    }
}

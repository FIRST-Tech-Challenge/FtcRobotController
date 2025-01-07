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
        extendo = new DoubleHorizontalExtendo(hardwareMap, "leftExtendo", "rightExtendo");
        controller = new GamepadEvents(gamepad1);
        waitForStart();
        while(opModeIsActive())
        {
            double extendoPower = controller.left_trigger.getTriggerValue() - controller.right_trigger.getTriggerValue();
            extendo.setPower(extendoPower);
            telemetry.addData("Left Extendo power: ", extendo.getLeftPower());
            telemetry.addData("Right Extendo power: ", extendo.getRightPower());
            telemetry.update();
            controller.update();
        }
    }
}

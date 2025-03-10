package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtendo;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "Extendo Test")
public class ExtendoTest extends LinearOpMode {
    GamepadEvents controller1;
    HorizontalExtendo extendo;
    @Override
    public void runOpMode() throws InterruptedException {
        controller1 = new GamepadEvents(gamepad1);
        extendo = new HorizontalExtendo("leftHExtendo", "rightHExtendo", true);
        waitForStart();
        while (opModeIsActive())
        {
            double pos = controller1.left_trigger.getTriggerValue() - controller1.right_trigger.getTriggerValue();
            extendo.move(pos);
            telemetry.addData("Exendo Pos", pos);
            controller1.update();
        }
    }
}

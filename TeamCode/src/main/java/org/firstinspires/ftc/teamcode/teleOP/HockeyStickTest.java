package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.HockeyStick;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
@TeleOp(name = "HockeyStick test")
public class HockeyStickTest extends LinearOpMode {
    GamepadEvents controller1;
    HockeyStick hockeyStick;
    @Override
    public void runOpMode() throws InterruptedException {
        controller1 = new GamepadEvents(gamepad1);
        hockeyStick = new HockeyStick(hardwareMap, "hockeyStick");

        waitForStart();
        while (opModeIsActive())
        {
            double pow = controller1.left_trigger.getTriggerValue() -controller1.right_trigger.getTriggerValue();
            hockeyStick.power(pow);
            telemetry.addData("Stick position: ", pow);
            controller1.update();
        }
    }
}

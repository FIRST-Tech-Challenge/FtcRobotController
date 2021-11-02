package org.firstinspires.ftc.teamcode.competition.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.competition.utils.teleopmanagers.GamepadFunctions;
import org.firstinspires.ftc.teamcode.competition.utils.teleopmanagers.MechanumTeleOpManager;

@TeleOp(name="LinearAutonomousTemplate", group="linear")

public class MechanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadFunctions function1 = new GamepadFunctions(gamepad1, true, false, false, false, false, false);
        GamepadFunctions function2 = new GamepadFunctions(gamepad2, false, false, false, false, false, false);
        MechanumTeleOpManager manager = new MechanumTeleOpManager(telemetry, hardwareMap, gamepad1, gamepad2, function1, function2);
        waitForStart();
        resetStartTime();
        while(opModeIsActive()) {
            manager.main();
        }
        manager.stop();
    }

}

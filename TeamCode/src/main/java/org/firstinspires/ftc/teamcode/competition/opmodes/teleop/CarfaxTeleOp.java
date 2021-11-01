package org.firstinspires.ftc.teamcode.competition.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.competition.utils.teleopmanagers.CarfaxTeleOpManager;
import org.firstinspires.ftc.teamcode.competition.utils.teleopmanagers.GamepadFunctions;

@TeleOp(name="CarfaxTeleOp", group="PostOpenHouseTeleOp")
public class CarfaxTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CarfaxTeleOpManager manager = new CarfaxTeleOpManager(telemetry, hardwareMap, gamepad1, gamepad2, new GamepadFunctions(gamepad1, true, true, true, true, true, true), new GamepadFunctions(gamepad2, true, true, true, true, true, true));
        waitForStart();
        resetStartTime();
        while(opModeIsActive()) {
            manager.main();
        }
        manager.stop();
    }

}


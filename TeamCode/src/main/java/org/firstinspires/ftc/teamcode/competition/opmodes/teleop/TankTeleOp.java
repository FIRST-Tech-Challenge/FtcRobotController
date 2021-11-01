package org.firstinspires.ftc.teamcode.competition.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.competition.utils.Tank;
import org.firstinspires.ftc.teamcode.competition.utils.teleopmanagers.GamepadFunctions;
import org.firstinspires.ftc.teamcode.competition.utils.teleopmanagers.TankTeleOpManager;

@TeleOp(name="TankOpMode", group="PostOpenHouseTeleOp")
public class TankTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TankTeleOpManager manager = new TankTeleOpManager(telemetry, hardwareMap, gamepad1, gamepad2, new GamepadFunctions(gamepad1, true, true, true, true, true, true), new GamepadFunctions(gamepad2, false, false, false, false, false, false));
        waitForStart();
        resetStartTime();
        while(opModeIsActive()) {
            manager.main();
        }
        manager.stop();
    }

}

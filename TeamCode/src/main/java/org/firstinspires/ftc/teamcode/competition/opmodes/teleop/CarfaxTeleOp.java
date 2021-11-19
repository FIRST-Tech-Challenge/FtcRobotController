package org.firstinspires.ftc.teamcode.competition.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.competition.utils.teleop.CarfaxTeleOpManager;
import org.firstinspires.ftc.teamcode.competition.utils.teleop.GamepadFunctions;
import org.firstinspires.ftc.teamcode.competition.utils.teleop.TeleOpHWDevices;

@TeleOp(name="CarfaxTeleOp", group="PostOpenHouseTeleOp")
public class CarfaxTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadFunctions gamepad1Functions = new GamepadFunctions(gamepad1, true, true, true, true, true, true);
        GamepadFunctions gamepad2Functions = new GamepadFunctions(gamepad2, false, false, false, false, false, false);
        TeleOpHWDevices devices = new TeleOpHWDevices(true, false, false, false, false, false, false);
        CarfaxTeleOpManager manager = new CarfaxTeleOpManager(telemetry, hardwareMap, gamepad1, gamepad2, gamepad1Functions, gamepad2Functions, devices);
        waitForStart();
        resetStartTime();
        while(opModeIsActive()) {
            manager.main();
        }
        manager.stop();
    }

}


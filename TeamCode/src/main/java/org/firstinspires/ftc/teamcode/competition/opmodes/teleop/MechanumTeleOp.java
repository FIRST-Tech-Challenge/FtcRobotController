package org.firstinspires.ftc.teamcode.competition.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.competition.utils.TeleOpEmergencyManager;
import org.firstinspires.ftc.teamcode.competition.utils.teleop.GamepadFunctions;
import org.firstinspires.ftc.teamcode.competition.utils.teleop.MechanumTeleOpManager;
import org.firstinspires.ftc.teamcode.competition.utils.teleop.TeleOpHWDevices;

@TeleOp(name="MechanumTeleOp", group="PostOpenHouseTeleOp")

public class MechanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TeleOpEmergencyManager emergencyManager = new TeleOpEmergencyManager(gamepad1, gamepad2);
        GamepadFunctions function1 = new GamepadFunctions(gamepad1, true, true, true, true, true, true);
        GamepadFunctions function2 = new GamepadFunctions(gamepad2, false, false, false, false, false, false);
        TeleOpHWDevices devices = new TeleOpHWDevices(true, false, true, false, false, false, false);
        MechanumTeleOpManager mechManager = new MechanumTeleOpManager(telemetry, hardwareMap, gamepad1, gamepad2, function1, function2, devices);
        waitForStart();
        resetStartTime();
        while(opModeIsActive() && emergencyManager.isSafe()) {
            mechManager.main();
        }
        mechManager.stop();
    }

}

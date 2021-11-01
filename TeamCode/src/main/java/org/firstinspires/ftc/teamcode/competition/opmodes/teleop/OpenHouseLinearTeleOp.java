package org.firstinspires.ftc.teamcode.competition.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.competition.utils.CarManager;

@TeleOp(name="OpenHouseTeleOp", group="competition")
public class OpenHouseLinearTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        resetStartTime();
        CarManager carManager = new CarManager(gamepad1, gamepad2, hardwareMap, telemetry);
        while(opModeIsActive()) {
            carManager.main();
        }
        carManager.stop();
        stop();
    }

}

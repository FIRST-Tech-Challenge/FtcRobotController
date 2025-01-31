package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ArmExtendTuningTest", group = "Tests")
public class ArmTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Arm arm = new Arm(this, true);
        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
        waitForStart();

        while (opModeIsActive()){
            TelemetryPacket packet = new TelemetryPacket();

            arm.setPowerAngleWithF(gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_Y));
            arm.setPowerExtendWithLimits(gamepadEx1.getStick(GamepadKeys.Stick.RIGHT_STICK_Y));

            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.update();

            gamepadEx1.update(gamepad1);

        }
    }
}
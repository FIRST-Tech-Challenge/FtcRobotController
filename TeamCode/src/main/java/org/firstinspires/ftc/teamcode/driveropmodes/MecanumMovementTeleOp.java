package org.firstinspires.ftc.teamcode.driveropmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardwaremaps.DeepHardwareMap;
import org.firstinspires.ftc.teamcode.Mecanum;

import org.firstinspires.ftc.teamcode.Helper;

/**
 * Class for testing of simple Mecanum movement
 */
@TeleOp(name="Mecanum Movement", group="Linear OpMode")
public class MecanumMovementTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Create hardware map
        DeepHardwareMap teamHardwareMap = new DeepHardwareMap(hardwareMap);

        // Gamepads to use for rising edge detector
        Gamepad current_gp = new Gamepad();
        Gamepad prev_gp = new Gamepad();

        // Power is 0.5
        Mecanum m = Mecanum.Init(
                teamHardwareMap.FrontRightMotor,
                teamHardwareMap.FrontLeftMotor,
                teamHardwareMap.BackRightMotor,
                teamHardwareMap.BackLeftMotor,
                0.5
                );

        // Make sure all motors are behaving properly
        Helper.ReportDriveMotorStatus(teamHardwareMap, telemetry);

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Copy both gamepads
            prev_gp.copy(current_gp);
            current_gp.copy(gamepad1);

            // Give gamepad to mecanum to move wheels
            // Display on telemetry
            Helper.ReportMecanumMotorSpeed(m.Move(gamepad1), telemetry);
            telemetry.update();
        }
    }
}
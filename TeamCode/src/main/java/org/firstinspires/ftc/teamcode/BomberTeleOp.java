package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Simple TeleOp for the MVP drivetrain.
 *
 * Physical wiring:
 *   LeftFrontMotor  - powers the left front wheel (steering + drive)
 *   RightFrontMotor - powers the right front wheel (steering + drive)
 *   BackMotor       - powers the single rear wheel (drive only, no steering)
 *
 * Controls (Gamepad 1):
 *   Left stick Y  - throttle (forward / reverse)
 *   Right stick X - steering (left / right)
 */
@TeleOp(name = "Bomber", group = "TeleOp")
public class BomberTeleOp extends LinearOpMode {
    private Drivetrain drivetrain;

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            drivetrain.drive(drive, turn);
        }

        drivetrain.stop();
    }
}

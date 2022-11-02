package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.ArmSystem;
import org.firstinspires.ftc.teamcode.opmodes.auto.CompetitionAutonomous;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

/**
 * Drives a pushbot with teleop control.
 */
@TeleOp(name = "Competition TeleOp", group="TeleOp")
public class CompetitionTeleOp extends BaseOpMode {

    /**
     * Drives the motors based on the joystick sticks
     * Left trigger engages slow-drive
     */
    public void loop() {
        float rx = (float) Math.pow(gamepad1.right_stick_x, 3);
        float lx = (float) Math.pow(gamepad1.left_stick_x, 3);
        float ly = (float) Math.pow(gamepad1.left_stick_y, 3);
        driveSystem.slowDrive(gamepad1.left_trigger > 0.3f);
        driveSystem.drive(rx, lx, ly);

        if (gamepad1.left_trigger > 0.1f) {
            armSystem.intake();
        }

        if (gamepad1.right_trigger > 0) {
            armSystem.outtake();
        }

        if (gamepad1.a) {
            // Move Bar to Low Position
            armSystem.driveToLevel(ArmSystem.LOW, 0.2);

        }

        if (gamepad1.b) {
            // Move Bar to Middle Position
            armSystem.driveToLevel(ArmSystem.MEDIUM, 0.2);
        }

        if (gamepad1.y) {
            // Move Bar to High Position
            armSystem.driveToLevel(ArmSystem.HIGH, 0.2);
        }

        if (gamepad1.x) {
            //
            armSystem.driveToLevel(ArmSystem.FLOOR, 0.2);
        }

        if (gamepad1.dpad_down) {
            // Move Bar Down Based on Time Pressed
        }

        if (gamepad1.dpad_up) {
            // Move Bar Up Based on Time Pressed
        }
    }

    public void placeCone(int level) {
        // Levels:
        // 1 = Low Goal
        // 2 = Mid Goal
        // 3 = High Goal

        level --; // Subtract 1 from level to make it usable in an array

        double[] liftLevels = {1 /* Low Goal */, 2 /* Mid Goal */, 3 /* High Goal */};

        // Lift 4Bar variable amount while aligning then drop disk
        align(pixycam.YELLOW, CompetitionAutonomous.POLE_WIDTH);
        //ArmSystem.fourbar.lift(liftLevels[level];
        //ArmSystem.outtake();
    }
}

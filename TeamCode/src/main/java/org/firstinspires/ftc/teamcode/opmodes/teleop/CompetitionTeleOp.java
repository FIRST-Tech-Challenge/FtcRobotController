package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.components.ArmSystem.Intake.State.INTAKING;
import static org.firstinspires.ftc.teamcode.components.ArmSystem.Intake.State.OUTTAKING;

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
        float rx = (float) Math.pow(gamepad2.right_stick_x, 3);
        float lx = (float) Math.pow(gamepad2.left_stick_x, 3);
        float ly = (float) Math.pow(gamepad2.left_stick_y, 3);

        driveSystem.slowDrive(gamepad1.left_bumper || ((armSystem.up())));

        driveSystem.drive(rx, lx, ly);
        telemetry.addData("arm Left encoder ", armSystem.armLeft.getCurrentPosition());
        telemetry.addData("arm right encoder ", armSystem.armRight.getCurrentPosition());
        telemetry.addData("arm left power",  armSystem.armLeft.getPower());
        telemetry.addData("arm right power", armSystem.armRight.getPower());


        if (gamepad1.left_trigger > 0 || armSystem.getState() == INTAKING) {
            armSystem.intake();
        }

        if(gamepad1.right_bumper){
            armSystem.toDaGround();
        }

        if (gamepad1.right_trigger > 0 || armSystem.getState() == OUTTAKING) {
            armSystem.outtake();
        }

        if (gamepad1.a) {
            // Move Bar to Low Position
            armSystem.driveToLevel(ArmSystem.LOW, 0.4);
        }

        if (gamepad1.b) {
            // Move Bar to Middle Position
            armSystem.driveToLevel(ArmSystem.MEDIUM, 0.8);
        }

        if (gamepad1.y) {
            // Move Bar to High Position
            armSystem.driveToLevel(ArmSystem.HIGH, 0.8);
        }

        if (gamepad1.x) {
            // Move Bar to Cone Position
            align(pixycam.YELLOW, CompetitionAutonomous.POLE_WIDTH);
        }

        if (gamepad1.dpad_down) {
            armSystem.killMotors();
        }

        if (gamepad1.dpad_up) {
            armSystem.driveToLevel(ArmSystem.FLOOR, 0.3);
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

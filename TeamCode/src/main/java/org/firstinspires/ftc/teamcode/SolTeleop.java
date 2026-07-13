package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "Sol", group = "TeleOp")
public class SolTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Shooter    shooter    = new Shooter(hardwareMap);
        Indexer    indexer    = new Indexer(hardwareMap);

        waitForStart();

        boolean prevB = false;

        while (opModeIsActive()) {
            boolean bPressed = gamepad1.b && !prevB;
            prevB = gamepad1.b;

            drivetrain.drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
            shooter.update(gamepad1);
            indexer.update(shooter.getRPM(), shooter.consumeFire(), bPressed);

            telemetry.addData("Shooter RPM",    "%.0f", shooter.getRPM());
            telemetry.addData("Target RPM",     "%.0f", shooter.getTargetRPM());
            telemetry.addData("Max Speed",       "%d%%", shooter.getMaxSpeedPercent());
            telemetry.addData("Shoot State",     shooter.getState());
            telemetry.addData("Turret Power",    "%.2f", shooter.getTurretPower());
            telemetry.addData("Turret Estimate", "%.2f", shooter.getTurretPosition());
            telemetry.update();
        }

        drivetrain.stop();
        shooter.stop();
        indexer.stop();
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Fighter", group = "TeleOp")
public class FighterTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Turret     turret     = new Turret(hardwareMap);
        Spindexer  spindexer  = new Spindexer(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
            turret.update(gamepad1);
            spindexer.update(turret.getRPM(), turret.consumeFire());

            telemetry.addData("Shooter RPM",    "%.0f", turret.getRPM());
            telemetry.addData("Max Speed",       "%d%%", turret.getMaxSpeedPercent());
            telemetry.addData("Shoot State",     turret.getState());
            telemetry.addData("Turret Position", "%.3f", turret.getTurretPosition());
            telemetry.update();
        }

        drivetrain.stop();
        turret.stop();
        spindexer.stop();
    }
}

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Aimer;

/**
 * Limelight turret-aiming test for the GREEN robot. All the aiming lives in {@link Aimer}.
 *
 * Driving (gamepad 1 sticks) works all the time, including while aiming.
 * Hold Y:        the TURRET turns to face the hive tags. Let go and it stops (and holds).
 * D-pad L/R:     nudge the turret slowly by hand. This IGNORES the soft limits so you can
 *                calibrate them -- watch the wires.
 *
 * During INIT: line the turret up facing straight backward, then press INIT. The turret
 * encoder is zeroed there. Press X for BLUE alliance or B for RED (default BLUE).
 *
 * Tuning values (gains, soft limits, tag IDs) are in Aimer.
 */
@TeleOp(name = "Limelight Test Green", group = "Test")
public class LimelightTestGreen extends GreenBotBaseOpMode {

    static final double TURRET_NUDGE_POWER = 0.15;  // d-pad manual nudge

    boolean wasOnTarget = false;

    @Override
    public void runOpMode() {
        config();   // also builds the aimer

        // Wherever the turret is right now (lined up facing backward) becomes 0.
        aimer.resetTurretEncoder();

        // Pick the alliance during INIT.
        while (opModeInInit()) {
            if (gamepad1.x) aimer.setBlueAlliance(true);
            if (gamepad1.b) aimer.setBlueAlliance(false);
            telemetry.addLine("Turret encoder zeroed -- it should be facing straight backward.");
            telemetry.addLine("Alliance: X = BLUE, B = RED");
            telemetry.addData("Alliance", aimer.isBlueAlliance() ? "BLUE" : "RED");
            telemetry.addLine("Hold Y to aim the turret. D-pad L/R nudges it.");
            telemetry.addData("Turret ticks", aimer.getTurretTicks());
            telemetry.update();
        }

        aimer.start();

        while (opModeIsActive()) {
            // Driving is always live -- that is the point of a turret.
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            drive.drive(forward, strafe, turn);

            aimer.update();

            if (gamepad1.dpad_left || gamepad1.dpad_right) {
                aimer.nudgeTurret(gamepad1.dpad_right ? TURRET_NUDGE_POWER : -TURRET_NUDGE_POWER);
            } else if (gamepad1.y) {
                aimer.aim();
            } else {
                aimer.stopTurret();
            }

            // Buzz the driver's controller once when we first line up.
            boolean onTarget = aimer.isOnTarget();
            if (onTarget && !wasOnTarget) {
                gamepad1.rumble(200);
            }
            wasOnTarget = onTarget;

            aimer.showStatus();
            telemetry.addData("Drive", "fwd: %.1f  strafe: %.1f  turn: %.1f", forward, strafe, turn);
            telemetry.update();
        }

        drive.stopDriving();
        aimer.stop();
    }
}

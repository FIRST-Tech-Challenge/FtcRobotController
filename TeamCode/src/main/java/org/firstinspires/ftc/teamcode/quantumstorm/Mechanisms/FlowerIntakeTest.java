package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/**
 * Bench test for the flower intake servo alone. Needs only "flower_servo" in the
 * active Robot Configuration, so it runs on a spare Control Hub with no drive motors.
 * Same buttons as QuantumStormTeleOp, so tuned values carry straight over, plus
 * bench-only A = DROP / Y = LIFT and a live gamepad readout for troubleshooting.
 * Hidden from the Driver Hub list; remove @Disabled to bench-test again.
 */
@Disabled
@TeleOp(name = "Test: Flower Intake Servo", group = "Quantum Storm Test")
public class FlowerIntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        FlowerIntake flowerIntake = new FlowerIntake(hardwareMap);

        // The arm moves to UP as soon as INIT is pressed
        flowerIntake.lift();

        telemetry.addLine("Flower arm moved to UP. Press START to test.");
        telemetry.addLine("Gamepad 2: BACK + D-Pad DOWN = DROP");
        telemetry.addLine("Gamepad 2: BACK + D-Pad UP = UP");
        telemetry.addLine("LB + D-Pad UP/DOWN = Nudge");
        telemetry.addLine("Bench only: A = DROP, Y = UP");
        telemetry.addLine("Gamepad not responding? Press START+B on it to make it User 2");
        telemetry.update();

        waitForStart();

        boolean previousA = gamepad2.a;
        boolean previousY = gamepad2.y;

        while (opModeIsActive()) {

            // Same controls as QuantumStormTeleOp
            flowerIntake.handleGamepad(gamepad2);

            // Bench-only single buttons, to rule out the combos while testing.
            // Ignored with START held, since START+B assigns the gamepad.
            if (!gamepad2.start) {
                if (gamepad2.a && !previousA) flowerIntake.drop();
                if (gamepad2.y && !previousY) flowerIntake.lift();
            }

            previousA = gamepad2.a;
            previousY = gamepad2.y;

            telemetry.addData(
                    "Flower Intake",
                    "%s  %.0f deg  (servo pos %.3f)",
                    flowerIntake.isDown() ? "DOWN" : "UP",
                    flowerIntake.getAngle(),
                    flowerIntake.getPosition());
            telemetry.addData("Limits", "%.0f .. %.0f deg",
                    Constants.FLOWER_ANGLE_MIN_DEG, Constants.FLOWER_ANGLE_MAX_DEG);
            telemetry.addLine("BACK+D-Pad DOWN/UP = DROP/UP,  LB+D-Pad = Nudge");
            telemetry.addLine("Bench only: A = DROP, Y = UP");

            // Live gamepad readout: if nothing here changes when buttons are
            // pressed, the gamepad is not assigned as User 2.
            telemetry.addLine("");
            telemetry.addData("Gamepad 2",
                    gamepad2.getGamepadId() == Gamepad.ID_UNASSOCIATED
                            ? "NOT ASSIGNED - press START+B on the gamepad"
                            : "assigned (" + gamepad2.type() + ")");
            telemetry.addData("Buttons",
                    "BACK %s  LB %s  UP %s  DOWN %s  A %s  Y %s",
                    onOff(gamepad2.back), onOff(gamepad2.left_bumper),
                    onOff(gamepad2.dpad_up), onOff(gamepad2.dpad_down),
                    onOff(gamepad2.a), onOff(gamepad2.y));
            telemetry.update();
        }
    }

    private static String onOff(boolean pressed) {
        return pressed ? "ON" : "-";
    }
}

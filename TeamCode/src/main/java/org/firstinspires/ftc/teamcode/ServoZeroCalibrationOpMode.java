package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
import java.util.List;

/** Raw programmer reference ONLY for one mechanically disconnected positional servo.
 * Installed mechanisms use Test: Turret / Gate / Depollinator with subsystem bounds.
 */
@TeleOp(name = "Calibrate: DISCONNECTED Servo RAW", group = "Calibration")
public class ServoZeroCalibrationOpMode extends LinearOpMode {
    @Override public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        List<Servo> servos = new ArrayList<>(hardwareMap.getAll(Servo.class));
        // The geared turret rotation pair is ONE selectable entry that always drives both servos with the
        // same direction, PWM range and position, so a forgotten coupling can never make them fight.
        Servo yaw1 = firstNamed(TurretSubsystem.ROTATION_SERVO_1_NAMES);
        Servo yaw2 = firstNamed(TurretSubsystem.ROTATION_SERVO_2_NAMES);
        PairedServos yawPair = null;
        if (yaw1 != null && yaw2 != null && yaw1 != yaw2) {
            yawPair = new PairedServos("Turret rotation servos", yaw1, yaw2, TurretSubsystem.ROTATION_SERVO_DIRECTION,
                    (int) PwmControl.PwmRange.defaultRange.usPulseLower, (int) PwmControl.PwmRange.defaultRange.usPulseUpper);
            servos.remove(yaw2);
        }
        int selected = 0;
        boolean previousLeft = false, previousRight = false;
        boolean armed = false, previousArm = false;
        while (opModeInInit() || opModeIsActive()) {
            loop.begin();
            boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
            if (!servos.isEmpty()) {
                if (left && !previousLeft) { selected = (selected + servos.size() - 1) % servos.size(); armed = false; }
                if (right && !previousRight) { selected = (selected + 1) % servos.size(); armed = false; }
                Servo servo = servos.get(selected);
                boolean arm = gamepad1.left_bumper && gamepad1.right_bumper;
                if (opModeIsActive() && arm && !previousArm) armed = !armed;
                if (!opModeIsActive()) armed = false;
                previousArm = arm;
                boolean pairSelected = yawPair != null && servo == yaw1;
                // Coupled turret servos are never exercised independently: the pair entry writes both.
                if (opModeIsActive() && armed && gamepad1.right_trigger > .8) {
                    double position = gamepad1.a ? .5 : gamepad1.x ? 0 : gamepad1.y ? 1 : Double.NaN;
                    if (pairSelected) yawPair.setPosition(position);
                    else if (Double.isFinite(position)) servo.setPosition(position);
                }
                if (pairSelected) {
                    telemetry.addData("Selected PAIR (both move together)", "%s + %s | %s | ports %d & %d",
                            hardwareMap.getNamesOf(yaw1), hardwareMap.getNamesOf(yaw2),
                            yaw1.getController().getConnectionInfo(), yaw1.getPortNumber(), yaw2.getPortNumber());
                } else {
                    telemetry.addData("Selected ONLY", "%s | %s | port %d", hardwareMap.getNamesOf(servo),
                            servo.getController().getConnectionInfo(), servo.getPortNumber());
                }
            }
            previousLeft = left; previousRight = right;
            telemetry.addLine("Disconnect selected servo from ALL linkages, including coupled turret gears.");
            telemetry.addLine("Installed mechanisms: use their bounded subsystem tests.");
            telemetry.addLine("Dpad L/R selects | AFTER START both bumpers toggle arm | hold RT + A/X/Y = .5/0/1");
            telemetry.addData("Raw calibration armed", armed);
            loop.addTelemetry(telemetry); telemetry.update(); idle();
        }
        // No return-to-center move at STOP: reference is unknown for a disconnected mechanism.
    }

    private Servo firstNamed(String[] names) {
        for (String name : names) {
            Servo servo = hardwareMap.tryGet(Servo.class, name);
            if (servo != null) return servo;
        }
        return null;
    }
}

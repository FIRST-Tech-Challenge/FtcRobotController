package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeLayers.DetectedColor;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.OneLayerIntake;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@TeleOp
public class MecanumAndTurretTest extends OpMode {
    // ---------- Drivetrain ----------
    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate;

    // ---------- Intake ----------
    OneLayerIntake intakeLayers = new OneLayerIntake();
    DetectedColor enemyColor = DetectedColor.RED;
    boolean manualGarden = false;
    boolean lastYState = false;
    boolean lastRightBumperState = false;
    boolean forceStop = false;

    // ---------- Turret ----------
    Turret turret = new Turret();
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;
    boolean manualShoot = false;
    boolean lastBState = false;

    // ---------- State Tracking (Tuning Controls) ----------
    boolean lastLeftTriggerState = false;
    boolean lastDpadLeftState = false;
    boolean lastDpadRightState = false;
    boolean lastDpadUpState = false;
    boolean lastDpadDownState = false;

    @Override
    public void init() {
        drive.init(hardwareMap);
        intakeLayers.init(hardwareMap);
        turret.init(hardwareMap, telemetry);
        telemetry.addLine("Initialized all mechanisms");
    }

    @Override
    public void start() {
        turret.resetTimer();
        intakeLayers.intake();
    }

    public void autoShoot(Turret turret) {
        turret.updateTagProcessor();
        List<AprilTagDetection> detections = turret.getTagByColor(enemyColor);
        turret.updateTurret(detections);

        telemetry.addData("Tuning P", turret.getkP());
        telemetry.addData("Tuning D", turret.getkD());
        telemetry.addData("Step Size", stepSizes[stepIndex]);

        if (!detections.isEmpty()) {
            telemetry.addLine("Turret: AprilTag Detected");
        } else {
            telemetry.addLine("Turret: No Tag Detected. Stopping Motor");
        }
    }

    @Override
    public void loop() {
        // ---------- Drivetrain ----------
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        drive.drive(forward, strafe, rotate);

        // ---------- Intake Logic ----------
        boolean enemyDetected = false;

        if (gamepad1.y && !lastYState) {
            manualGarden = !manualGarden;
        }
        lastYState = gamepad1.y;
        if (gamepad1.right_bumper && !lastRightBumperState) {
            forceStop = !forceStop;
        }
        lastRightBumperState = gamepad1.right_bumper;

        if (forceStop) {
            intakeLayers.stop();
        } else if (enemyDetected || manualGarden) {
            intakeLayers.garden();
        } else {
            intakeLayers.intake();
        }

        // ---------- Turret Tuning Controls ----------
        boolean currentLeftTriggerState = gamepad1.left_trigger > 0.0;
        if (currentLeftTriggerState && !lastLeftTriggerState) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        lastLeftTriggerState = currentLeftTriggerState;

        if (gamepad1.dpad_left && !lastDpadLeftState) {
            turret.setkP(turret.getkP() - stepSizes[stepIndex]);
        }
        lastDpadLeftState = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !lastDpadRightState) {
            turret.setkP(turret.getkP() + stepSizes[stepIndex]);
        }
        lastDpadRightState = gamepad1.dpad_right;

        if (gamepad1.dpad_up && !lastDpadUpState) {
            turret.setkD(turret.getkD() + stepSizes[stepIndex]);
        }
        lastDpadUpState = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !lastDpadDownState) {
            turret.setkD(turret.getkD() - stepSizes[stepIndex]);
        }
        lastDpadDownState = gamepad1.dpad_down;

        // ---------- Turret Execution ----------
        if (gamepad1.b && !lastBState) {
            manualShoot = !manualShoot;
        }
        lastBState = gamepad1.b;

        if (!manualShoot) {
            autoShoot(turret);
        } else {
            turret.manualMode();
        }
    }
}

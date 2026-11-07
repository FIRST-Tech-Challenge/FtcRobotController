package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.FlowerExtraction;
import org.firstinspires.ftc.teamcode.mechanisms.FlowerHood;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeLayers;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeLayers.DetectedColor;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@TeleOp
public class TeleOpMode extends OpMode {
    // ---------- Drivetrain ----------
    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate;

    // ---------- Intake ----------
    IntakeLayers intakeLayers = new IntakeLayers();
    DetectedColor enemyColor = DetectedColor.RED;
    boolean manualGarden = false;
    boolean lastYState = false;

    boolean lastRightBumperState = false;
    boolean forceStop = false;

    // ---------- Flower Extraction ----------
    FlowerExtraction flowerExtraction = new FlowerExtraction();
    boolean extractorDown = false;
    boolean lastXState = false;

    // ---------- Flower Hood ----------
    FlowerHood flowerHood = new FlowerHood();
    boolean hoodDown = true;
    boolean lastLBState = false;

    // ---------- Turret ----------
    Turret turret = new Turret();

    // Tuning P and D
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;
    boolean manualShoot = false;
    boolean lastBState = false;

    // State tracking variables for edge-detection tuning controls
    boolean lastLeftTriggerState = false;
    boolean lastDpadLeftState = false;
    boolean lastDpadRightState = false;
    boolean lastDpadUpState = false;
    boolean lastDpadDownState = false;

    @Override
    public void init() {
        drive.init(hardwareMap);
        intakeLayers.init(hardwareMap);
        flowerExtraction.init(hardwareMap);
        flowerHood.init(hardwareMap);
        turret.init(hardwareMap, telemetry);
        telemetry.addLine("Initialized all mechanisms");
    }

    @Override
    public void start() {
        intakeLayers.intake();
        flowerExtraction.up();
        flowerHood.down();
        turret.resetTimer();
    }

    public void autoShoot(Turret turret) {
        turret.updateTagProcessor();
        List<AprilTagDetection> detections = turret.getTagByColor(enemyColor);
        turret.updateTurret(detections);
//        telemetry.addData("Tuning P", turret.getkP());
//        telemetry.addData("Tuning D", turret.getkD());
//        telemetry.addData("Step Size", stepSizes[stepIndex]);
//
//        if (!detections.isEmpty()) {
//            telemetry.addLine("Turret: AprilTag Detected");
//        } else {
//            telemetry.addLine("Turret: No Tag Detected. Stopping Motor");
//        }
    }

    @Override
    public void loop() {
        // ---------- Drivetrain ----------
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        drive.drive(forward, strafe, rotate);

        // ---------- Intake ----------
        DetectedColor outerColorL = intakeLayers.getDetectedColor(telemetry, 'L');
        DetectedColor outerColorR = intakeLayers.getDetectedColor(telemetry, 'R');
        boolean enemyDetected = outerColorL == enemyColor || outerColorR == enemyColor;

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

        // ---------- Flower extraction ----------
        if (gamepad1.x && !lastXState) {
            if (extractorDown) {
                flowerExtraction.up();
            } else {
                flowerExtraction.down();
            }
            extractorDown = !extractorDown;
        }
        lastXState = gamepad1.x;

        // ---------- Flower hood ----------
        if (gamepad1.left_bumper && !lastLBState) {
            if (hoodDown) {
                flowerHood.up();
            } else {
                flowerHood.down();
            }
            hoodDown = !hoodDown;
        }
        lastLBState = gamepad1.left_bumper;

        // ---------- Turret Tuning Controls (Handled once per loop frame) ----------
        // Left trigger threshold logic acts like a button press
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
            turret.shoot(turret.getDetectedColor(telemetry));
        } else {
            turret.manualMode();
            turret.shoot(turret.getDetectedColor(telemetry));
        }
    }
}

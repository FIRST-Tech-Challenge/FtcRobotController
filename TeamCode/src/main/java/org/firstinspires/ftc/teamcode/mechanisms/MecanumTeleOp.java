package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import android.util.Size;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    boolean init = false;
    double farP = 13;
    double farF = 15;
    double farV = 2041.8773;
    double closeP = 73;
    double closeF = 14;
    double closeV = 1500;
    double curTargetV = closeV;
    double hoodPos = 0;
    double gateStart = System.currentTimeMillis();
    double gateElapsed() { return System.currentTimeMillis() - gateStart; }

    // PID
    double kP = 0.03;
    double kD = 0.001;
    double lastError = 0;

    private double headingPID(double error) {
        double derivative = error - lastError;
        lastError = error;
        return (kP * error) + (kD * derivative);
    }

    @Override
    public void runOpMode() {

        // ---------------- MOTORS ----------------
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BR");

        DcMotorEx shooterMotor1 = hardwareMap.get(DcMotorEx.class, "S1");
        DcMotorEx shooterMotor2 = hardwareMap.get(DcMotorEx.class, "S2");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("NTK");

        CRServo gate = hardwareMap.get(CRServo.class, "Gate");
        Servo hood = hardwareMap.get(Servo.class, "Hood");

        // Directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(closeP, 0, 0, closeF);
        shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // ---------------- APRILTAG (LOW POWER) ----------------
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // lowest power
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480)) // lowest power
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .build();

        // Start OFF
        visionPortal.setProcessorEnabled(aprilTag, true);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---------------- MAIN LOOP ----------------
        while (opModeIsActive()) {
            if (!init) {
                hoodPos = 0;
                hood.setPosition(0);
                init = true;
            }}}}
            // -------- Vision Toggle --------
     /*       AprilTagDetection targetTag = null;

            if (gamepad1.right_bumper || gamepad2.right_trigger >= 0.05) {
                for (AprilTagDetection tag : aprilTag.getDetections()) {
                    if (tag.id == 20 || tag.id == 24) {
                        targetTag = tag;
                        break;
                    }
                }
            }

            // -------- Driving --------
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            // Snap-to-target
            if (gamepad1.right_bumper && targetTag != null) {

                double error = targetTag.ftcPose.bearing;

                if (Math.abs(error) > 0.5) {

                    double output = headingPID(error);

                    if (Math.abs(output) < 0.08) {
                        output = Math.signum(output) * 0.08;
                    }

                    rx = Math.max(-0.7, Math.min(0.7, output));

                } else {
                    rx = 0;
                }

                telemetry.addData("Snap", "ACTIVE");
                telemetry.addData("Tag", targetTag.id);
            } else {
                telemetry.addData("Snap", "NO TAG");
                telemetry.addData("Tag", "---");
            }

            /*if (hoodPos + 0.01 <= 1 && gamepad2.right_stick_y > 0.05) {
                hoodPos += 0.01;
                hood.setPosition(hoodPos);
            } else if (hoodPos - 0.01 >= 0 && gamepad2.right_stick_y < -0.05) {
                hoodPos -= 0.01;
                hood.setPosition(hoodPos);
            }*/

      /*      if (gamepad2.right_trigger >= 0.05 && targetTag != null) {
                double dist = targetTag.ftcPose.range;
                if (dist >= 90) {
                    hoodPos = 1;
                    hood.setPosition(hoodPos);
                }

                telemetry.addData("Distance to Goal", Math.round(dist));

            } else if (gamepad2.right_trigger >= 0.05 && targetTag == null) {
                hoodPos = 0;
                hood.setPosition(hoodPos);
                telemetry.addData("Distance to Goal", "VOID");
            } else {
                hoodPos = 0;
                telemetry.addData("Distance to Goal", "VOID");
            }
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double fl = (y + x + rx) / denominator;
            double bl = (y - x + rx) / denominator;
            double fr = (y - x - rx) / denominator;
            double br = (y + x - rx) / denominator;

            double driveMult = 1;
            if (gamepad1.left_trigger > 0.05) driveMult = 0.75;
            if (gamepad1.right_trigger > 0.05) driveMult = 1.5;

            frontLeftMotor.setPower(fl * driveMult);
            backLeftMotor.setPower(bl * driveMult);
            frontRightMotor.setPower(fr * driveMult);
            backRightMotor.setPower(br * driveMult);

            if (gamepad2.left_bumper) {
                gate.setPower(.25);
            } else if (gamepad2.left_trigger > .05) {
                gate.setPower(-.25);
            } else {
                gate.setPower(0);
            }

            // Intake
            if (gamepad2.dpad_down) intakeMotor.setPower(-1);
            else if (gamepad2.dpad_up) intakeMotor.setPower(1);
            else intakeMotor.setPower(0);

            telemetry.addData("Hood Position", Math.round(hoodPos));

            if (hoodPos == 1) {
                pidf = new PIDFCoefficients(farP, 0, 0, farF);
                curTargetV = farV;
            } else {
                pidf = new PIDFCoefficients(closeP, 0, 0, closeF);
                curTargetV = closeV;
            }
            shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

            hood.setPosition(hoodPos);
            shooterMotor1.setVelocity(curTargetV);
            shooterMotor2.setVelocity(curTargetV);

            telemetry.update();
        }
    }
}*/
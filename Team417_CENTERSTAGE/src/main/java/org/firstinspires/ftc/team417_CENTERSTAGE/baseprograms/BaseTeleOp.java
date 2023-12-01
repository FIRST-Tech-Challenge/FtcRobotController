package org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.team417_CENTERSTAGE.apriltags.AprilTagPoseEstimator;
import org.firstinspires.ftc.team417_CENTERSTAGE.mechanisms.ArmTeleOp;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

@Config
public abstract class BaseTeleOp extends BaseOpMode {
    public AprilTagPoseEstimator myAprilTagPoseEstimator;
    public MecanumDrive drive;
    private ArmTeleOp arm;

    @Override
    public void runOpMode() {
        myAprilTagPoseEstimator = new AprilTagPoseEstimator(this);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        initializeHardware();

        myAprilTagPoseEstimator.init();
        arm = new ArmTeleOp(gamepad2, armMotor, dumperServo);

        resetDumper();

        waitForStart();

        while (opModeIsActive()) {
            driveUsingControllers(false);

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);

            // Code added to draw the pose, remove before competition, causes lags:
            TelemetryPacket p = new TelemetryPacket();
            Canvas c = p.fieldOverlay();
            c.setStroke("#3F5100");
            MecanumDrive.drawRobot(c, drive.pose);

            myAprilTagPoseEstimator.updatePoseEstimate();

            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(p);

            if (armMotor != null && dumperServo != null && gateServo != null) {
                outputUsingControllers();
                intakeUsingControllers();
                telemetry.addData("ArmMotor", armMotor.getCurrentPosition());
                telemetry.addData("DumperServo", dumperServo.getPosition());
                telemetry.addData("GateServo", gateServo.getPosition());
            }
            telemetry.addData("FRMotor", FR.getCurrentPosition());
            telemetry.addData("FRMotor", FR.getPowerFloat());
            telemetry.addData("FLMotor", FL.getCurrentPosition());
            telemetry.addData("FLMotor", FL.getPowerFloat());
            telemetry.addData("BRMotor", BR.getCurrentPosition());
            telemetry.addData("BRMotor", BR.getPowerFloat());
            telemetry.addData("BLMotor", BL.getCurrentPosition());
            telemetry.addData("BLMotor", BL.getPowerFloat());
            telemetry.update();
        }

        // Close camera to avoid errors
        myAprilTagPoseEstimator.visionPortal.close();
    }

    public boolean sensitive = false;

    public void driveUsingControllers() {
        sensitive = gamepad1.right_bumper;

        double sensitivity, rotSensitivity;
        double strafeConstant = 1.1;

        if (sensitive) {
            sensitivity = 0.5;
            rotSensitivity = 0.8;
        } else {
            sensitivity = 1;
            rotSensitivity = 1;
        }

        double x = curveStick(gamepad1.left_stick_x) * strafeConstant * sensitivity;
        double y = curveStick(-gamepad1.left_stick_y) * sensitivity;
        double rot = curveStick(gamepad1.right_stick_x) * rotSensitivity;

        mecanumDrive(x, y, rot);
    }

    public void driveUsingControllers(boolean curve) {
        sensitive = gamepad1.right_bumper;

        double sensitivity, rotSensitivity;
        double strafeConstant = 1.1;

        if (sensitive) {
            sensitivity = 0.5;
            rotSensitivity = 0.8;
        } else {
            sensitivity = 1;
            rotSensitivity = 1;
        }

        double x, y, rot;
        if (curve) {
            x = curveStick(gamepad1.left_stick_x) * strafeConstant * sensitivity;
            y = curveStick(-gamepad1.left_stick_y) * sensitivity;
            rot = curveStick(gamepad1.right_stick_x) * rotSensitivity;
        } else {
            x = gamepad1.left_stick_x * strafeConstant * sensitivity;
            y = -gamepad1.left_stick_y * sensitivity;
            rot = gamepad1.right_stick_x * rotSensitivity;
        }
        mecanumDrive(x, y, rot);
    }

    public void intakeUsingControllers() {
        double speed = gamepad1.left_trigger - gamepad1.right_trigger;
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("Intake Speed", speed);
        runIntakeMechanism(speed);
    }

    public void outputUsingControllers() {
        controlDumperUsingControllers();
        controlGateUsingControllers();
        arm.armControl();
    }

    public boolean dumperDumped = false;
    public boolean bIsPressed = false;
    public boolean yIsPressed = false;

    public void controlDumperUsingControllers() {
        if (!bIsPressed && gamepad2.b) {
            if (dumperDumped) {
                dumpDumper();
            } else {
                resetDumper();
            }
            dumperDumped = !dumperDumped;
        }

        bIsPressed = gamepad2.b;

        if (!yIsPressed && gamepad2.y) {
            if (dumperDumped) {
                tiltDumper();
            } else {
                resetDumper();
            }
            dumperDumped = !dumperDumped;
        }

        yIsPressed = gamepad2.y;
    }

    public boolean gateOpen = false;
    public boolean aIsPressed = false;

    public void controlGateUsingControllers() {
        if (!aIsPressed && gamepad2.a){
            if (gateOpen) {
                closeGate();
            } else {
                openGate();
            }
            gateOpen = !gateOpen;
        }
        aIsPressed = gamepad2.a;
    }

    //Adds stick curve to the joysticks
    public double curveStick(double rawSpeed) {
        double logSpeed;
        if (rawSpeed >= 0) {
            logSpeed = Math.pow(rawSpeed, 2);
        } else {
            logSpeed = -Math.pow(rawSpeed, 2);
        }
        return logSpeed;
    }
}

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class Launcher {
    private static final int NEAR_SHOT = 1;
    private static final int MID_SHOT = 2;
    private static final int FAR_SHOT = 3;
    private static final int nearVelocity = 1300;
    private static final int midVelocity = 1650;
    private static final int farVelocity = 1900;
    private static final double nearAngle = 0.0;
    private static final double midAngle = 0.60;
    private static final double farAngle = 0.62;
    private static final double nearDistance = 0.0;
    private static final double midDistance = 70.0;
    private MecanumOpMode lox;
    private Gamepad gamepad1;
    private Telemetry telemetry;
    private DcMotorEx flywheel;
    private Servo servo;
    private DcMotor coreHex;
    private CRServo intake;
    private static final double F = 14.098; // Feedforward gain to counteract constant forces like friction.
    private static final double P = 265;    // Proportional gain to correct error based on how far off the velocity is.

    public Launcher(MecanumOpMode lox) {
        this.lox = lox;
    }


    public void init(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        servo = hardwareMap.get(Servo.class, "servo");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        // intake =hardwareMap.get(CRServo.class, "intake" );
    }

    public void manualFeederControl() {
        // Manual control for the Core Hex intake
        if (gamepad1.cross) {
            coreHex.setPower(1);
        } else if (gamepad1.triangle) {
            coreHex.setPower(-1);
        }
    }

    public void shotButtons() {
        if (gamepad1.left_bumper) {
            farShot();
        } else if (gamepad1.right_bumper) {
            midShot();
        } else if (gamepad1.square) {
            nearShot();
            //} else if (gamepad1.right_trigger>0) {
            //    intake.setPower(10);
        } else if (gamepad1.circle) {
            autoShot();
        } else {
            flywheel.setVelocity(0);
            coreHex.setPower(0);
        }

        telemetry.addData("Flywheel Velocity", flywheel.getVelocity());
        telemetry.addData("Flywheel Power", flywheel.getPower());
    }

    public void midShot() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheel.setVelocity(midVelocity);
        servo.setPosition(midAngle);
        if (flywheel.getVelocity() >= midVelocity - 100) {
            coreHex.setPower(-1);
        } else {
            coreHex.setPower(0);
        }
    }

    public void nearShot() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheel.setVelocity(nearVelocity);
        servo.setPosition(nearAngle);
        if ((flywheel.getVelocity() >= nearVelocity)) {
            coreHex.setPower(-1);
        } else {
            coreHex.setPower(0);
        }
    }

    /**
     * The far power velocity is intended for launching balls a few feet from the goal. It may require adjusting the deflector.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The servo will spin until the bumper is released.
     */
    public void farShot() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheel.setVelocity(farVelocity);
        servo.setPosition(farAngle);
        if (flywheel.getVelocity() >= farVelocity - 100) {
            coreHex.setPower(-1);
        } else {
            coreHex.setPower(0);
        }
    }

    public void shootWithTime(int shotType, int timeout) {

        ElapsedTime shotTimer = new ElapsedTime();
        while (lox.opModeIsActive() && shotTimer.milliseconds() < timeout) {
            if (shotType == NEAR_SHOT) {
                nearShot();
            } else if (shotType == MID_SHOT) {
                midShot();
            } else if (shotType == FAR_SHOT) {
                farShot();
            } else {

            }
        }

        //Stop the power
        flywheel.setPower(0);
        coreHex.setPower(0);
    }

    private void autoShot() {
        //autoshot will adjust what velocity and angle it shoots at depending on how far it is
        // from the goal and it will get this data from the camera code.
    }

//    private void autoShot() {
//        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
//        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//
//        double distance = 0.0;
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                distance = detection.ftcPose.y;
//            }
//        }
//
//        double angle = (midAngle - nearAngle) / (midDistance - nearDistance) * distance + nearAngle;
//        double velocity = (midVelocity - nearVelocity) / (midDistance - nearDistance) * distance + nearVelocity;
//
//        telemetry.addData("angle: ", angle);
//
//        flywheel.setVelocity(velocity);
//        servo.setPosition(angle);
//        if (flywheel.getVelocity() >= velocity - 100) {
//            coreHex.setPower(1);
//        } else {
//            coreHex.setPower(0);
//        }
//    }

}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class MecanumDriveTrain {
    private MecanumOpMode lox;
    private Telemetry telemetry;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private static final float slowSpeed = 0.3F;
    private final double WHEELS_INCHES_TO_TICKS = (28 * 5 * 3) / (3 * Math.PI);
    private final ElapsedTime autoDriveTimer = new ElapsedTime();

    public MecanumDriveTrain(MecanumOpMode linearOpMode) {
        this.lox = linearOpMode;
    }


    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Correct way if motors are configured correctly
//        if (true) {
//            frontLeft.setDirection(DcMotor.Direction.REVERSE);
//            backLeft.setDirection(DcMotor.Direction.REVERSE);
//            frontRight.setDirection(DcMotor.Direction.FORWARD);
//            backRight.setDirection(DcMotor.Direction.FORWARD);
//        }

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveWithGamepad(Gamepad gamepad1) {
        //there is a negative value so that we can fix the fact that when we tilt the joystick
        // forward the telemetry shows a negative value.
        double forwardBack = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        telemetry.addData("forwardBack:", forwardBack);
        telemetry.addData("strafe:", strafe);
        telemetry.addData("turn:", turn);

        float power = 1;
        if (gamepad1.left_trigger > 0) {
            power = slowSpeed;
        }

        if (gamepad1.right_trigger > 0) {
            AprilTagDetection detection = lox.camera.findAprilTag(20);
            if (detection != null) {
                double bearing = detection.ftcPose.bearing;
                turn = Range.clip(-bearing * 0.04, -0.5, 0.5);
                lox.telemetry.addData("Bearing:", bearing);
                lox.telemetry.update();
            }
        }

        drive(forwardBack, strafe, turn, power);
    }

    public void driveWithTime(double forward, double strafe, double turn, double power, int timeout) {
        drive(forward, strafe, turn, power);

        ElapsedTime driveTimer = new ElapsedTime();
        while (lox.opModeIsActive() && driveTimer.milliseconds() < timeout) {
            lox.idle();
        }

        // Stop the power
        drive(0,0,0,0);
    }

    public void drive(double forward, double strafe, double turn, double power) {
        double leftFrontPower = forward + strafe + turn;
        double rightFrontPower = forward - strafe - turn;
        double leftBackPower = forward - strafe + turn;
        double rightBackPower = forward + strafe - turn;
        // Setting Motor Power
        frontLeft.setPower(leftFrontPower * power);
        frontRight.setPower(rightFrontPower * power);
        backLeft.setPower(leftBackPower * power);
        backRight.setPower(rightBackPower * power);
    }

}
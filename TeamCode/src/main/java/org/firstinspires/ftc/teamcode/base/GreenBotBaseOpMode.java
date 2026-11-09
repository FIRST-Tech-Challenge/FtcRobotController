package org.firstinspires.ftc.teamcode.base;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Aimer;
import org.firstinspires.ftc.teamcode.robot.Drive;

public abstract class GreenBotBaseOpMode extends LinearOpMode {

    protected DcMotor frontLeft, frontRight, backLeft, backRight;
    protected DcMotorEx intake, turret, launcher, feeder;
    protected CRServo leftIntakeServo, rightIntakeServo;

    protected GoBildaPinpointDriver pinpoint;
    protected Limelight3A limelight;
    protected Drive drive;
    protected Aimer aimer;

    /**
     * Configures the hardware that all OpModes using this robot share.
     * Call this first thing on startup.
     */
    public void config() {
        // Instantiate hardware
        frontLeft = hardwareMap.dcMotor.get("front_left");
        frontRight = hardwareMap.dcMotor.get("front_right");
        backLeft = hardwareMap.dcMotor.get("back_left");
        backRight = hardwareMap.dcMotor.get("back_right");
        intake = hardwareMap.get(DcMotorEx.class, "intake_motor");
        turret = hardwareMap.get(DcMotorEx.class, "turret_motor");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        feeder = hardwareMap.get(DcMotorEx.class, "feeder_motor");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Configure hardware
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(DcMotor.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(BRAKE);
        frontLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        pinpoint.setOffsets(-2.44, -6.56, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5) // adjust these for spinup and recovery
        );

        drive = new Drive(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                pinpoint,
                telemetry
        );

        // Does NOT zero the turret encoder: each OpMode decides when the turret's position
        // should count as 0 (call aimer.resetTurretEncoder()).
        aimer = new Aimer(turret, limelight, telemetry);
    }

}

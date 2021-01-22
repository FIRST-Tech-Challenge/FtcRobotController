package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public abstract class UltimateGoalHardware extends RobotHardware {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public static final double SHOOTER_POWER = 0.5535;

    public enum UltimateGoalStartingPosition  {
        LEFT,
        RIGHT
    }

    public enum WobbleGoalDestination {
        A,
        B,
        C
    }

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor shooter;
    public DcMotor collector;
    public DcMotor escalator;

    public DcMotor wobbleGoalHolder;
    public Servo wobbleServo;

    public final static double COUNTS_PER_ENCODER_REV = 8192;
    public final static double WHEEL_DIAMETER_IN = 4;

    @Override
    public void initializeHardware() {
        frontLeft = this.initializeDevice(DcMotor.class, "frontLeft");
        frontRight = this.initializeDevice(DcMotor.class, "frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = this.initializeDevice(DcMotor.class, "backLeft");
        backRight = this.initializeDevice(DcMotor.class, "backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter = this.initializeDevice(DcMotor.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        collector = this.initializeDevice(DcMotor.class, "collector");
        collector.setDirection(DcMotorSimple.Direction.REVERSE);
        escalator = this.initializeDevice(DcMotor.class, "escalator");
        //.setDirection(DcMotorSimple.Direction.REVERSE);
        wobbleGoalHolder = this.initializeDevice(DcMotor.class, "wobble");
        wobbleServo = this.initializeDevice(Servo.class, "wobbleServo");
        wobbleServo.setPosition(0);
        revIMU = this.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        revIMU.initialize(parameters);

//        wobbleGoalLeftClaw = this.initializeDevice(Servo.class, "leftClaw");
//        wobbleGoalLeftClaw.scaleRange(0.375, 0.55);
//        wobbleGoalRightClaw = this.initializeDevice(Servo.class, "rightClaw");
//        wobbleGoalRightClaw.scaleRange(0.8,0.95);

        this.initializeOmniDrive(frontLeft, frontRight, backLeft, backRight);
        this.omniDrive.setCountsPerInch(COUNTS_PER_ENCODER_REV/(Math.PI*WHEEL_DIAMETER_IN));
    }

    @Override
    public void initializeLocalizer() {
        super.initializeLocalizer();
        //this.localizer.setRobotStart(revIMU, 90);
        this.localizer.encodersXScaleFactor = 40.0/48.0; // ANTI JANK
        this.localizer.loadUltimateGoalTrackables(this);
        this.localizer.setCameraMatrix(this,
                new Position(DistanceUnit.INCH, 9.5, 0, 0, 0),
                new Orientation(EXTRINSIC, YZX, DEGREES, -90, 0, 0, 0));
    }

    @Override
    public void initTfod() {
        super.initTfod();
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}

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

    public static final double SHOOTER_POWER = 0.5235;
    public static final double SHOOTER_RPM = 2700;
    public static final double SHOOTER_RPM_THRESHOLD = 300;
    public static final double SHOOTER_POWER_INCREMENT = 0.03;
    public static final double SHOOTER_POWER_FINE_INCREMENT = 0.005;
    public static final double SHOOTER_POWER_FINE_INCREMENT_RANGE = 1000;
    boolean spinShooter = false;
    double currentShooterPower = 0;
    double currentShooterRPM = 0;
    double targetShooterRPM = SHOOTER_RPM;
    long shooterPrevTime = System.currentTimeMillis();
    int shooterPrevPos = 0;

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
    public final static double WHEEL_DIAMETER_IN = 4.0;

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
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collector = this.initializeDevice(DcMotor.class, "collector");
        escalator = this.initializeDevice(DcMotor.class, "escalator");
        //.setDirection(DcMotorSimple.Direction.REVERSE);
        wobbleGoalHolder = this.initializeDevice(DcMotor.class, "wobble");
//        wobbleGoalHolder.setTargetPosition(wobbleGoalHolder.getCurrentPosition() + 72); // 72 = 90deg
//        wobbleGoalHolder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        wobbleGoalHolder.setPower(1);
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
        this.localizer.loadUltimateGoalTrackables(this,
                new Position(DistanceUnit.INCH, -9.25, 0, 0, 0),
                new Orientation(EXTRINSIC, YZX, DEGREES, -90, 0, 0, 0));

    }

    @Override
    public void hardware_loop() {
        super.hardware_loop();

        long current_time = System.currentTimeMillis();
        int current_pos = shooter.getCurrentPosition();
        int deltaPos = current_pos - shooterPrevPos;
        long deltaTime = current_time - shooterPrevTime;
        shooterPrevPos = current_pos;
        shooterPrevTime = current_time;
        currentShooterRPM = (deltaPos/28.0) / (deltaTime) * (1000*60);

        if (spinShooter) {
            if (Math.abs(currentShooterRPM - targetShooterRPM) > SHOOTER_RPM_THRESHOLD) {
                double increment = Math.abs(currentShooterRPM - targetShooterRPM) <= SHOOTER_POWER_FINE_INCREMENT_RANGE ? SHOOTER_POWER_FINE_INCREMENT : SHOOTER_POWER_INCREMENT;

                if (currentShooterRPM < targetShooterRPM) {
                    // too slow
                    currentShooterPower = Math.min(currentShooterPower + increment, 1);
                } else {
                    // too fast
                    targetShooterRPM = SHOOTER_RPM;
                    currentShooterPower = Math.max(currentShooterPower - increment, 0);
                }
            }
        } else {
            currentShooterPower = 0;
        }

        shooter.setPower(currentShooterPower);
        telemetry.addData("Current Shooter Power", currentShooterPower);
        telemetry.addData("Target RPM", targetShooterRPM);
        telemetry.addData("RPM", currentShooterRPM);
    }

    public void setShooterEnabled(boolean enabled) {
        if (enabled && !this.spinShooter) {
            this.targetShooterRPM = SHOOTER_RPM - 1000;
        }
        this.spinShooter = enabled;
    }

    public boolean canShoot() {
        return Math.abs(this.currentShooterRPM - this.SHOOTER_RPM) <= SHOOTER_RPM_THRESHOLD;
    }

    @Override
    public void initTfod() {
        super.initTfod();
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}

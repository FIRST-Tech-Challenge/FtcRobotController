package org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

@Config
public abstract class BaseOpMode extends LinearOpMode {
    MecanumDrive drive;

    //Declares LEDs on DevBot
    public DigitalChannel red;
    public DigitalChannel green;

    public DcMotor intakeMotor ;
    public DcMotor armMotor ;
    static final public double ARM_MOTOR_MIN_POSITION = 0;
    static final public double ARM_MOTOR_MAX_POSITION = 4200;
    public Servo dumperServo;
    public static final double DUMPER_SERVO_TILT_POSITION = 0.4;
    public static final double DUMPER_SERVO_RESET_POSITION = 0.527;
    public static final double DUMPER_SERVO_DUMP_POSITION = 0.2;
    public Servo gateServo;
    public final double GATE_SERVO_OPEN_POSITION = 0;
    public final double GATE_SERVO_CLOSE_POSITION = 0.55;

    //Initializes motors, servos, and sensors
    public void initializeHardware() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        //Drive Motors, other motors, sensors, etc.
        if(MecanumDrive.isDevBot) {
            red = initializeDigitalChannel("red", DigitalChannel.Mode.OUTPUT);
            green = initializeDigitalChannel("green", DigitalChannel.Mode.OUTPUT);

            // Turn LEDs off
            //    DigitalChannel object for LEDs makes this counterintuitive, on = false, off = true
            red.setState(true);
            green.setState(true);
        } else {
            //Mechanism Motors
            intakeMotor = initializeMotor("IntakeMotor", DcMotor.Direction.FORWARD);
            armMotor = initializeMotor("ArmMotor", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);//DcMotor.RunMode.RUN_TO_POSITION);

            //Mechanism Servos
            dumperServo = initializeServo("DumperServo", Servo.Direction.FORWARD);
            gateServo = initializeServo("GateServo", Servo.Direction.FORWARD);
        }
    }

    // A digital channel is a device that can accept either a 1 or a 0 of input.
    // In this case "green" and "red" are LED lights.
    public DigitalChannel initializeDigitalChannel(String channelName, DigitalChannel.Mode mode) {
        DigitalChannel digitalChannel = hardwareMap.get(DigitalChannel.class, channelName);
        digitalChannel.setMode(mode);
        return digitalChannel;
    }

    public DcMotor initializeMotor(String motorName, DcMotorSimple.Direction direction, DcMotor.RunMode mode) {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(mode);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
        return motor;
    }

    public DcMotor initializeMotor(String motorName, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
        return motor;
    }

    public Servo initializeServo(String servoName, Servo.Direction direction) {
        Servo servo = hardwareMap.get(Servo.class, servoName);
        servo.setDirection(direction);
        return servo;
    }

    public void mecanumDrive(double x, double y, double rot) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1);

        double frontLeftPower = (y + x + rot) / denominator;
        double frontRightPower = (y - x - rot) / denominator;
        double backLeftPower = (y - x + rot) / denominator;
        double backRightPower = (y + x - rot) / denominator;


        drive.leftFront.setPower(frontLeftPower);
        drive.rightFront.setPower(frontRightPower);
        drive.leftBack.setPower(backLeftPower);
        drive.rightBack.setPower(backRightPower);
    }

    public void runIntakeMechanism(double speed) {
        intakeMotor.setPower(speed);
    }

    public void tiltDumper() {
        dumperServo.setPosition(DUMPER_SERVO_TILT_POSITION);
    }

    public void resetDumper() {
        dumperServo.setPosition(DUMPER_SERVO_RESET_POSITION);
    }

    public void dumpDumper() {
        dumperServo.setPosition(DUMPER_SERVO_DUMP_POSITION);
    }

    enum DumperAction {
        DUMPING,
        RESETTING,
        STOPPING
    }
    public void moveDumper(DumperAction dumperAction) {
        switch (dumperAction) {
            case DUMPING:
                dumperServo.setPosition(1);
            case RESETTING:
                dumperServo.setPosition(0);
            case STOPPING:
                dumperServo.setPosition(dumperServo.getPosition());
            default:
        }
    }

    public void openGate() {
        gateServo.setPosition(GATE_SERVO_OPEN_POSITION);
    }

    public void closeGate() {
        gateServo.setPosition(GATE_SERVO_CLOSE_POSITION);
    }

    public double[] armPositions = new double[] {ARM_MOTOR_MIN_POSITION, ARM_MOTOR_MIN_POSITION + ((ARM_MOTOR_MAX_POSITION - ARM_MOTOR_MIN_POSITION) / 2), ARM_MOTOR_MAX_POSITION};

    public void moveArm(double speed) {
        if (armMotor.getCurrentPosition() > ARM_MOTOR_MAX_POSITION) {
            speed = -0.1;
        } else if (armMotor.getCurrentPosition() < ARM_MOTOR_MIN_POSITION) {
            speed = 0.1;
        }
        armMotor.setPower(speed);
    }

    public final static double EPSILON = 0.0001;
    public static boolean isEpsilonEquals(double a, double b) {
        return (Math.abs(a) + EPSILON >= Math.abs(b) && Math.abs(a) - EPSILON <= Math.abs(b));
    }
}

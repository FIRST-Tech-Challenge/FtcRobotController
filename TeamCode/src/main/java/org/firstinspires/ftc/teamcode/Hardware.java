package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.AutoClearEncoder;
import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.hardware.EncoderFor;
import org.firstinspires.ftc.teamcode.hardware.HardwareMapper;
import org.firstinspires.ftc.teamcode.hardware.HardwareName;
import org.firstinspires.ftc.teamcode.hardware.MotorSet;
import org.firstinspires.ftc.teamcode.hardware.Reversed;
import org.firstinspires.ftc.teamcode.hardware.ZeroPower;
import org.firstinspires.ftc.teamcode.mmooover.TriOdoProvider;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.Servo;

import dev.aether.collaborative_multitasking.SharedResource;


public class Hardware extends HardwareMapper implements TriOdoProvider {
    public static final double spinTickPerRev = 751.8;
    public static final double RIGHT_SLIDE_OUT = 0.72;
    public static final double RIGHT_SLIDE_IN = 0.45;
    public static final double SLIDE_INWARD_TIME = 0.75; // seconds
    public static final double SLIDE_OUTWARD_TIME = 0.45; // seconds
    public static final double SLIDE_OVERSHOOT = 0.05;
    public static final double FLIP_DOWN = 0.05;
    public static final double FRONT_OPEN = 0.25;
    public static final double FRONT_CLOSE = 0.07;
    public static final double FLIP_UP = 0.98;
    public static final double CLAW_CLOSE = 0.28;
    public static final double CLAW_OPEN = 0.5;
    public static final double WRIST_UP = 0.42;
    public static final double WRIST_BACK = 0.30;
    public static final double ARM_POWER = 0.2;
    public static final double LAMP_BLUE = 0.611;
    public static final double LAMP_RED = 0.28;
    public static final double LAMP_ORANGE = 0.333;
    public static final double LAMP_YELLOW = 0.36;
    public static final double LAMP_PURPLE = 0.700;

    public static int deg2arm(double degrees) {
        return (int) (degrees / 360.0 * spinTickPerRev);
    }

    public static class Locks {
        /// The four drive motors: frontLeft, frontRight, backLeft, and backRight.
        public static final SharedResource DriveMotors = new SharedResource("DriveMotors");

        /// The vertical slide.
        ///
        /// It usually makes sense to have one task 'own' this the entire time
        /// and provide its own APIs and lock.
        public static final SharedResource VerticalSlide = new SharedResource("VerticalSlide");

        /// The components that make up the main arm assembly:
        /// * the `arm` motor
        /// * the `wrist` and `claw` servos
        public static final SharedResource ArmAssembly = new SharedResource("ArmAssembly");

        /// The `horizontalSlide` and `horizontalLeft` servos.
        public static final SharedResource HorizontalSlide = new SharedResource("HorizontalSlide");

        public static final SharedResource HSlideClaw = new SharedResource("HSlideClaw");
    }

    public static final double TRACK_WIDTH = 11.3385888;
    public static final double FORWARD_OFFSET = 5.05905785;
    public static final double ENC_WHEEL_RADIUS = 1.25984 / 2.0;
    public static final int ENC_TICKS_PER_REV = 2000;

    // left = left motor = exp 0 frontLeft
    // right = right motor = ctr 0 frontRight
    // center = ctr 3 intake

    @HardwareName("frontLeft")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor frontLeft;

    @HardwareName("frontRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor frontRight;

    @HardwareName("backLeft")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    @Reversed
    public DcMotor backLeft;

    @HardwareName("backRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor backRight;

    @HardwareName("verticalSlides")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor verticalSlide;

    @HardwareName("verticalSlides")
    @AutoClearEncoder
    public DcMotor encoderVerticalSlide;

    @HardwareName("arm")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    @AutoClearEncoder
    public DcMotor arm;

    @EncoderFor("frontLeft")
    @AutoClearEncoder
    @Reversed
    public Encoder encoderLeft;

    @EncoderFor("backRight")
    @AutoClearEncoder
    @Reversed
    public Encoder encoderCenter;

    @EncoderFor("frontRight")
    @AutoClearEncoder
    public Encoder encoderRight;

    @HardwareName("gyro")
    public NavxMicroNavigationSensor gyro;

    @HardwareName("claw")
    public Servo claw;

    @HardwareName("wrist")
    public Servo wrist;

    @HardwareName("clawFront")
    public Servo clawFront;

    @HardwareName("clawFlip")
    public Servo clawFlip;

    @HardwareName("horizontalSlide")
    public Servo horizontalSlide;

    @HardwareName("horizontalLeft")
    public Servo horizontalLeft;

    @HardwareName("lightLeft")
    public Servo lightLeft;

    @HardwareName("lightRight")
    public Servo lightRight;

    @HardwareName("clawColor")
    public ColorSensor clawColor;

    @Override
    public Encoder getLeftEncoder() {
        return encoderLeft;
    }

    @Override
    public Encoder getRightEncoder() {
        return encoderRight;
    }

    @Override
    public Encoder getCenterEncoder() {
        return encoderCenter;
    }
    // Values calculated from [very scuffed] CAD measurements.


    @Override
    public double getTrackWidth() {
        return TRACK_WIDTH;
    }

    @Override
    public double getForwardOffset() {
        return FORWARD_OFFSET;
    }

    @Override
    public double getEncoderWheelRadius() {
        return ENC_WHEEL_RADIUS;
    }

    @Override
    public int getEncoderTicksPerRevolution() {
        return ENC_TICKS_PER_REV;
    }

    public MotorSet driveMotors;

    public Hardware(HardwareMap hwMap) {
        super(hwMap);
        driveMotors = new MotorSet(
                frontLeft,
                frontRight,
                backLeft,
                backRight
        );
    }

}

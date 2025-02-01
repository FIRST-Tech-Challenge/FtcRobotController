package org.firstinspires.ftc.teamcode;
//spider
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Devices {
    final double CLAW_OPEN = 0.750;
    final double CLAW_CLOSED = 1.05;

        final double MIN_SLIDER_TICKS = 10; // chosen just to be safe

    /**
     *  This is the maximum length the slider is allowed to be in order
     *  to fit within the required dimensions, when the slider is pointed
     *  horizontally.
     */
        final double MAX_HORIZONTAL_SLIDER_TICKS = 2200;
//         final double MAX_SLIDER_INCHES = 24.25;
        final double ARM_ANGLE_TICKS_PER_DEGREE = 39.0;

        /** This is the angle that the arm starts each match */
        final double STARTING_ANGLE = -7;

        final double MAX_SAFE_SLIDER_TICKS = 3050; // needs to be tested

        DcMotorEx wormGear;
        DcMotorEx sliderMotor;
        DcMotorEx leftFrontDrive;
        DcMotorEx rightFrontDrive;
        DcMotorEx rightBackDrive;
        DcMotorEx leftBackDrive;
        Servo clawServo;
        BHI260IMU imu;

    public void init(HardwareMap hwmap) {

        wormGear = hwmap.get(DcMotorEx.class, "wormGear");

        imu = hwmap.get(BHI260IMU.class, "imu");
        imu.initialize();

        sliderMotor = hwmap.get(DcMotorEx.class, "sliderMotor");

        leftFrontDrive = hwmap.get(DcMotorEx.class, "leftFront");
        rightFrontDrive = hwmap.get(DcMotorEx.class, "rightFront");
        leftBackDrive = hwmap.get(DcMotorEx.class, "leftBack");
        rightBackDrive = hwmap.get(DcMotorEx.class, "rightBack");

        clawServo = hwmap.get(Servo.class, "clawServo");

        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//https://www.youtube.com/watch?v=dQw4w9WgXcQ
    }

    double wormGearTicks() {
        return wormGear.getCurrentPosition();
    }

    double wormGearAngle() {
        return (wormGearTicks() / ARM_ANGLE_TICKS_PER_DEGREE) + STARTING_ANGLE;
    }

    double wormGearRadians() {
        return Math.toRadians(wormGearAngle());
    }

    double upperMaxLegalSliderLength() {
        return MAX_HORIZONTAL_SLIDER_TICKS / Math.cos(wormGearRadians());
    }

    /**
     * lowerMaxSliderLength stops the slider from hitting the ground
     * @return
     */
    double lowerMaxSliderLength() {
        return 475.1 / Math.cos((0.5 * Math.PI) + wormGearRadians());
    }
//Robot Height is approx 1341.3 in ticks
}

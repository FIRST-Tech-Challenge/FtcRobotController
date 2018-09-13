package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SampleMecanumDrive extends MecanumDrive {
    // TODO: change your drive motor
    public static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);

    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    /**
     * These were good velocity PID values for a ~40lb robot with 1:1 belt-driven wheels off AM
     * orbital 20s. Adjust accordingly (or tune them yourself, see
     * https://github.com/acmerobotics/relic-recovery/blob/master/TeamCode/src/main/java/com/acmerobotics/relicrecovery/opmodes/tuner/DriveVelocityPIDTuner.java
     */
    public static final PIDCoefficients NORMAL_VELOCITY_PID = new PIDCoefficients(20, 8, 12);

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        // TODO: this needs to be tuned using FeedforwardTuningOpMode
        super(1);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        for (DcMotorEx motor : Arrays.asList(leftFront, leftRear, rightRear, rightFront)) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, at least tune kStatic and kA potentially
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, NORMAL_VELOCITY_PID);
        }

        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
    }

    private static double encoderTicksToInches(int ticks) {
        // TODO: modify this appropriately
        // wheel radius * radians/rev * wheel revs/motor revs * motor revs
        return 2 * 2 * Math.PI * 1.0 * ticks / TICKS_PER_REV;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            wheelPositions.add(encoderTicksToInches(motors.get(i).getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }
}

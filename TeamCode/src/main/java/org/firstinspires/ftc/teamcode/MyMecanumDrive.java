package org.firstinspires.ftc.teamcode;

import com.acmerobotics.splinelib.drive.MecanumDrive;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

public class MyMecanumDrive extends MecanumDrive {
    public static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);

    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    public static final PIDCoefficients NORMAL_VELOCITY_PID = new PIDCoefficients(20, 8, 12);

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    public MyMecanumDrive(HardwareMap hardwareMap) {
        super(6.82);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        for (DcMotorEx motor : Arrays.asList(leftFront, leftRear, rightRear, rightFront)) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, NORMAL_VELOCITY_PID);
        }

        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private static double encoderTicksToInches(int ticks) {
        return 2 * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    @NotNull
    @Override
    public List<Double> getMotorPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftFront.getCurrentPosition()),
                encoderTicksToInches(leftRear.getCurrentPosition()),
                encoderTicksToInches(rightRear.getCurrentPosition()),
                encoderTicksToInches(rightFront.getCurrentPosition())
        );
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }
}

package com.hfrobots.tnt.season1819;

import android.util.Log;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.hfrobots.tnt.corelib.Constants;
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

public class RoadrunnerMecanumDriveAdapter extends MecanumDrive {

    public static final double MOTOR_MAX_RPM = 340.0; // NR Orbital 20
    private static final double TICKS_PER_REV = 537.6; // NR Orbital 20

    /**
     * These were good velocity PID values for a ~40lb robot with 1:1 belt-driven wheels off AM
     * orbital 20s. Adjust accordingly (or tune them yourself, see
     * https://github.com/acmerobotics/relic-recovery/blob/master/TeamCode/src/main/java/com/acmerobotics/relicrecovery/opmodes/tuner/DriveVelocityPIDTuner.java
     */
    public static final PIDCoefficients NORMAL_VELOCITY_PID = new PIDCoefficients(20, 8, 12);

    public static final double TRACK_WIDTH = 18;

    public static final double WHEEL_BASE = 12;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    public RoadrunnerMecanumDriveAdapter(HardwareMap hardwareMap) {
        // TODO: this needs to be tuned using FeedforwardTuningOpMode
        super(TRACK_WIDTH, WHEEL_BASE, 0, 0);

        setLocalizer(new MecanumLocalizer(this, false));

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFrontDriveMotor");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRearDriveMotor");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRearDriveMotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFrontDriveMotor");

        for (DcMotorEx motor : Arrays.asList(leftFront, leftRear, rightRear, rightFront)) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, at least tune kStatic and kA potentially
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //motor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, NORMAL_VELOCITY_PID);
        }

        // No assumptions about motor directions
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
    }

    public RoadrunnerMecanumDriveAdapter(final DcMotorEx leftFront, final DcMotorEx leftRear,
                                         final DcMotorEx rightRear, final DcMotorEx rightFront) {
        super(TRACK_WIDTH, WHEEL_BASE,0, 0);

        this.leftFront = leftFront;
        this.leftRear = leftRear;
        this.rightRear = rightRear;
        this.rightFront = rightFront;

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
    }

    private static double encoderTicksToInches(int ticks) {
        // wheel radius * radians/rev * wheel revs/motor revs * motor revs
        //return 2 * 2 * Math.PI * 1.0 * ticks / TICKS_PER_REV;

        return ticks / 37; // empirically measured
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositionsInches = new ArrayList<>();
        List<Integer> wheelTicks = new ArrayList<>();

        for (int i = 0; i < 4; i++) {
            wheelPositionsInches.add(encoderTicksToInches(motors.get(i).getCurrentPosition()));
            wheelTicks.add(motors.get(i).getCurrentPosition());
        }

        Log.d(Constants.LOG_TAG, "Pos: lf: " + wheelTicks.get(0) + ", lr: "
                + wheelTicks.get(1) + ", rr: " + wheelTicks.get(2) + ", rf: " + wheelTicks.get(3));

        return wheelPositionsInches;
    }

    @Override
    public void setMotorPowers(double leftFront, double leftRear, double rightRear, double rightFront) {
        Log.d(Constants.LOG_TAG, "Power lf: " + leftFront + ", lr: " + leftRear + ", rr: "
                + rightRear + ", rf:" + rightFront);
        this.leftFront.setPower(leftFront);
        this.leftRear.setPower(leftRear);
        this.rightRear.setPower(rightRear);
        this.rightFront.setPower(rightFront);
    }

    @Override
    protected double getRawExternalHeading() {
        return 0;
    }
}
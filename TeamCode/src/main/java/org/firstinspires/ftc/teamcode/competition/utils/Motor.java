package org.firstinspires.ftc.teamcode.competition.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Represents a motor attached to the robot.
 * @author Thomas Ricci, Mickael Lachut
 * */
public class Motor {

    private final Telemetry TELEMETRY;
    private final String NAME;
    private final HardwareMap HARDWARE;
    private final DcMotor MOTOR;
    private final DcMotorSimple.Direction OFFSET;
    private final double COUNTS_PER_REV;
    private final double GEAR_REDUCTION;
    private final double RADIUS;
    private final double COUNTS_PER_INCH;
    private final MotorType TYPE;

    private enum MotorType {
        SIMPLE, 
        COMPLEX
    }


    /**
     * Creates a reference to a complex motor on the robot. This motor can drive to a position, unlike the simple variant.
     * @param telemetry The telemetry object to log data to.
     * @param hardware The hardware object to locate the motor with.
     * @param name The name of the motor as listed on the FtcRobotController device.
     * @param offset The directional offset of the motor. This can be useful if a motor is mounted the opposite way it should be, for example upside down.
     * @param countsPerRev The amount of encoder steps per motor revolution.
     * @param gearReduction The reduction ratio of the motor's gearing.
     * @param radius The radius of the motor's attachment.
     */
    public Motor(Telemetry telemetry, HardwareMap hardware, String name, DcMotorSimple.Direction offset, double countsPerRev, double gearReduction, double radius) {
        TELEMETRY = telemetry;
        NAME = name;
        HARDWARE = hardware;
        MOTOR = HARDWARE.get(DcMotor.class, name);
        OFFSET = offset;
        COUNTS_PER_REV = countsPerRev;
        GEAR_REDUCTION = gearReduction;
        RADIUS = radius;
        COUNTS_PER_INCH = (COUNTS_PER_REV * GEAR_REDUCTION) / (RADIUS * 2 * Math.PI);
        MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR.setDirection(offset);
        MOTOR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TYPE = MotorType.COMPLEX;
        TELEMETRY.addLine("Motor " + getName() + " is ready.");
    }

    /**
     * Creates a reference to a simple motor on the robot This motor can't drive a distance, but can drive with and without the encoder.
     * @param telemetry The telemetry object to log data to.
     * @param hardware The hardware object to locate the motor with.
     * @param name The name of the motor as listed on the FtcRobotController device.
     * @param offset The directional offset of the motor. This can be useful if a motor is mounted the opposite way it should be, for example upside down.
     */
    public Motor(Telemetry telemetry, HardwareMap hardware, String name, DcMotorSimple.Direction offset) {
        TELEMETRY = telemetry;
        NAME = name;
        HARDWARE = hardware;
        MOTOR = hardware.get(DcMotor.class, name);
        OFFSET = offset;
        COUNTS_PER_REV = 0;
        GEAR_REDUCTION = 0;
        RADIUS = 0;
        COUNTS_PER_INCH = 0;
        MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR.setDirection(offset);
        MOTOR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TYPE = MotorType.SIMPLE;
        TELEMETRY.addLine("Motor " + getName() + " is ready.");

    }

    /**
     * Drives the motor a certain distance.
     * @param distance The distance to drive in inches.
     * @param speed The maximum speed of the motor. The speed may be anywhere between -1 and this value, depending on where the motor is. This value cannot be below 0 though.
     * @throws IllegalArgumentException The error to throw when the maximum speed is not between -100 and 100.
     * @throws IllegalStateException The error to throw when the motor's TYPE != MotorType.COMPLEX.
     */
    public void driveDistance(int distance, int speed) throws IllegalArgumentException, IllegalStateException {
        if(speed < -100 || speed > 100) {
            throw new IllegalArgumentException("Speed is out of bounds!");
        }else if(TYPE == MotorType.SIMPLE) {
            throw new IllegalStateException("Motor type " + TYPE + " is simple, but must be complex for this method!");
        }
        MOTOR.setTargetPosition(MOTOR.getCurrentPosition() + (int)(distance * getCountsPerInch()));
        MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double realSpeed = speed / 100.0;
        MOTOR.setPower(realSpeed);
    }

    /**
     * Drives the motor at a certain speed.
     * @param speed The speed to set the motor to.
     * @throws IllegalArgumentException The error thrown when the speed is not between -100 and 100.
     */
    public void driveWithEncoder(int speed) throws IllegalArgumentException {
        if(speed < -100 || speed > 100) {
            throw new IllegalArgumentException("Speed is out of bounds!");
        }
        MOTOR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double realSpeed = speed / 100.0;
        MOTOR.setPower(realSpeed);
    }

    /**
     * Sends a certain voltage to the motor.
     * @param power The voltage to send to the motor.
     * @throws IllegalArgumentException The error thrown when the voltage is not between -100 and 100.
     */
    public void driveWithoutEncoder(int power) throws IllegalArgumentException {
        if(power < -100 || power > 100) {
            throw new IllegalArgumentException("Power is out of bounds!");
        }
        MOTOR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double realPower = power / 100.0;
        MOTOR.setPower(realPower);
    }

    /**
     * Bring the motor to a stop and reset the encoder.
     */
    public void stop() {
        brake();
        reset();
    }

    /**
     * Bring the motor to a stop.
     */
    public void brake() {
        MOTOR.setPower(0);
    }

    /**
     * Reset the encoder.
     */
    public void reset() {
        MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public Telemetry getTelemetry() {
        return TELEMETRY;
    }

    public String getName() {
        return NAME;
    }

    public String toString() {
        return NAME;
    }

    public HardwareMap getHardware() {
        return HARDWARE;
    }

    public DcMotor getDcMotor() {
        return MOTOR;
    }

    public DcMotorSimple.Direction getOffset() {
        return OFFSET;
    }

    public double getCountsPerRev() {
        return COUNTS_PER_REV;
    }

    public double getGearReduction() {
        return GEAR_REDUCTION;
    }

    public double getRadius() {
        return RADIUS;
    }

    public double getCountsPerInch() {
        return COUNTS_PER_INCH;
    }
    
    public MotorType getType() {
        return TYPE;
    }

}

package org.firstinspires.ftc.teamcode.utils.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private final ElapsedTime TIME = new ElapsedTime();

    /**
     * Creates a reference to a motor on the robot.
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
        TELEMETRY.addData("Status", "Resetting Encoders");    //
        TELEMETRY.update();
        MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MOTOR.setDirection(offset);
        TELEMETRY.addData("Encoder reset and location found",  "Position is ", MOTOR.getCurrentPosition());
        TELEMETRY.update();
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

    public ElapsedTime getRuntime() {
        return TIME;
    }

}

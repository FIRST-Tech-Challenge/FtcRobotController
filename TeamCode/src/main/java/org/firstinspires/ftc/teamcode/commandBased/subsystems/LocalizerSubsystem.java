package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.rr.util.Encoder;

public class LocalizerSubsystem extends SubsystemBase {

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.74803; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 0; // X is the up and down direction
    public static double PARALLEL_Y = 0; // Y is the strafe direction

    public static double PERPENDICULAR_X = 0;
    public static double PERPENDICULAR_Y = 0;

    private final RevIMU imu;

    private double heading;

    private Encoder parallelEncoder, perpendicularEncoder;

    public LocalizerSubsystem(final HardwareMap hwMap){
        imu = new RevIMU(hwMap);
        imu.init();

        parallelEncoder = new Encoder(hwMap.get(DcMotorEx.class, "parallelEncoder"));
        perpendicularEncoder = new Encoder(hwMap.get(DcMotorEx.class, "perpendicularEncoder"));
    }

    @Override
    public void periodic() {

    }

    public double getHeading() {
        heading = imu.getRotation2d().getDegrees();
        return heading;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public double getParallelEncoder() {
        return encoderTicksToInches(parallelEncoder.getCurrentPosition());
    }

    public double getPerpendicularEncoder() {
        return encoderTicksToInches(perpendicularEncoder.getCurrentPosition());
    }
}

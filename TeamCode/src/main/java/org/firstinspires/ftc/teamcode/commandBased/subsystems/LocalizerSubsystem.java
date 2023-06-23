package org.firstinspires.ftc.teamcode.commandBased.subsystems;

//import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commandBased.classes.Vector2d;

import org.firstinspires.ftc.teamcode.rr.util.Encoder;

@Disabled
public class LocalizerSubsystem extends SubsystemBase {

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.74803; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 1.37795; // X is the up and down direction
    public static double PARALLEL_Y = -5.029438; // Y is the strafe direction

    public static double PERPENDICULAR_X = 1.39764;
    public static double PERPENDICULAR_Y = 5.24311;

    public static double X_90_DEGREES = 0.319111944;
    public static double Y_90_DEGREES = -0.168413344;

    public static double parallelInches;
    public static double perpendicularInches;

    public static double pastParallelInches;
    public static double pastPerpendicularInches;

    //private final RevIMU imu;

    private double heading;

    private final Encoder parallelEncoder;
    private final Encoder perpendicularEncoder;

    private Vector2d relativePosition;
    private Vector2d positionChange;
    private Vector2d absolutePosition;

    public LocalizerSubsystem(final HardwareMap hwMap){
//        imu = new RevIMU(hwMap);
//        imu.init();

        parallelEncoder = new Encoder(hwMap.get(DcMotorEx.class, "parallelEncoder"));
        perpendicularEncoder = new Encoder(hwMap.get(DcMotorEx.class, "perpendicularEncoder"));

        relativePosition = new Vector2d(0, 0);
        positionChange = new Vector2d(0, 0);
        absolutePosition = new Vector2d(0, 0);
    }

    @Override
    public void periodic() {
        parallelInches = encoderTicksToInches(parallelEncoder.getCurrentPosition());
        perpendicularInches = encoderTicksToInches(perpendicularEncoder.getCurrentPosition());

        positionChange = new Vector2d(parallelInches - pastParallelInches, perpendicularInches - pastPerpendicularInches);

        //positionChange = positionChange.rotateBy(-getHeading());

        relativePosition = relativePosition.add(positionChange);

        pastParallelInches = parallelInches;
        pastPerpendicularInches = perpendicularInches;
    }

//    public double getHeading() {
//        heading = imu.getRotation2d().getDegrees();
//        return heading;
//    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public double getParallelEncoder() {
        return parallelInches;
    }

    public double getPerpendicularEncoder() {
        return perpendicularInches;
    }

    public double getPastParallelEncoder() {
        return pastParallelInches;
    }

    public double getPastPerpendicularEncoder() {
        return pastPerpendicularInches;
    }

    public Vector2d getPosition() {
        return relativePosition;
    }

    public double getPositionX() {
        return relativePosition.getX();
    }

    public double getPositionY() {
        return relativePosition.getY();
    }

    public double getChangeX() {
        return positionChange.getX();
    }

    public double getChangeY() {
        return positionChange.getY();
    }
}

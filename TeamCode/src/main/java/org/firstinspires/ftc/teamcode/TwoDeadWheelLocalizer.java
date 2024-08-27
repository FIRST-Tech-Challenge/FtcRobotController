package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.TwoDeadWheelInputsMessage;

@Config
public final class TwoDeadWheelLocalizer implements Localizer {
    public static class Params {

        // 30 = 2004.5
        public double parYTicks = 2004.5; // y position of the parallel encoder (in tick units)

        // 30 = 130.1
        public double perpXTicks = 130.1; // x position of the perpendicular encoder (in tick units)

        // used to correct IMU inaccuracy when turning multiple times in the same direction
        // adjust as needed
        public double headingMultiplier = 0.004555; // custom variable | 0.002755 LOGO UP USB FORWARD
    }

    public static Params PARAMS = new Params();

    public final Encoder par, perp;
    public final IMU imu;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;


    public final double headingRange = Math.PI - (-Math.PI) + 1; // custom variable
    public final double degree30 = 0.5235; // custom variable
    private double accumulativeYaw; // custom variable
    public int turnCount; // custom variable
    private double previousYaw; // custom variable


    private final double inPerTick;

    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;

    public TwoDeadWheelLocalizer(HardwareMap hardwareMap, IMU imu, double inPerTick) {

        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "backLeft")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontRight")));

        par.setDirection(DcMotorSimple.Direction.REVERSE);
        perp.setDirection(DcMotorSimple.Direction.REVERSE);

        this.imu = imu;

        this.inPerTick = inPerTick;

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);

        this.accumulativeYaw = 0; // custom
        this.turnCount = 0; // custom
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);


        FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));



        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));
        // custom variable
        double currentYaw = heading.toDouble();


        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        double rawHeadingVel = angularVelocity.zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading.plus(-(turnCount*PARAMS.headingMultiplier));
            previousYaw = currentYaw; // custom

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }




        // determine turnCount by using the accumulated heading from turning
        double deltaYaw = currentYaw - previousYaw;
        if (deltaYaw < -Math.PI) {
            deltaYaw += (Math.PI*2);
        } else if (deltaYaw >= Math.PI) {
            deltaYaw -= (Math.PI*2);
        }
        accumulativeYaw += deltaYaw;

        // turnCount increases roughly every 30 degrees (slightly undershoots)
        turnCount = (int) (accumulativeYaw / degree30);


        // adjust the heading based on the modifier
        heading = heading.plus(-(turnCount*PARAMS.headingMultiplier));
        double correctedHeadingVal = heading.toDouble();

        // ensure heading is within range of actual IMU readings
        // (wraps the number around -PI and PI) | (Math.PI) = max, (-Math.PI) = min
        if (correctedHeadingVal > Math.PI) {
            heading = Rotation2d.exp((-Math.PI) + (correctedHeadingVal - Math.PI) % headingRange);
        } else if (correctedHeadingVal < (-Math.PI)) {
            heading = Rotation2d.exp(Math.PI - ((-Math.PI) - correctedHeadingVal) % headingRange);
        }


        // END

        double headingDelta = heading.minus(lastHeading);

        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;


        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;
        previousYaw = currentYaw; // custom

        return twist;
    }
}

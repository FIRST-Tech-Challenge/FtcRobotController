package org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated;

import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.inPerTick_Calculation;
import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.parYTicks;
import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.perpXTicks;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public final class TwoDeadWheelLocalizer {

    public final Encoder par, perp;
    public final IMU imu;
    public double currentVelocity;
    public double currentRotationalVelocity;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    private final double inPerTick;

    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;

    public TwoDeadWheelLocalizer(HardwareMap hardwareMap, RobotMap robotMap) {
        par = robotMap.getParallel();
        perp = robotMap.getPerpendicular();
        imu = robotMap.getIMU();

        this.inPerTick = inPerTick_Calculation();
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public double getCurrentRotationalVelocity(){
        return currentRotationalVelocity;
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();
        currentVelocity = Math.hypot(parPosVel.rawVelocity, perpPosVel.rawVelocity);

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocityDegrees = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        currentRotationalVelocity = angularVelocityDegrees.xRotationRate;
        AngularVelocity angularVelocity = new AngularVelocity(
                UnnormalizedAngleUnit.RADIANS,
                (float) Math.toRadians(angularVelocityDegrees.xRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.yRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.zRotationRate),
                angularVelocityDegrees.acquisitionTime
        );

        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

        double rawHeadingVel = angularVelocity.xRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - parYTicks * headingDelta,
                                parPosVel.velocity - parYTicks * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - perpXTicks * headingDelta,
                                perpPosVel.velocity - perpXTicks * headingVel,
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

        return twist;
    }
}

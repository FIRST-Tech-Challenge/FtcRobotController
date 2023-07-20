package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.RawEncoder;

@Config
public final class TwoDeadWheelLocalizer implements Localizer {
    public static double PAR_Y_TICKS = 0.0;
    public static double PERP_X_TICKS = 0.0;

    public final Encoder par, perp;
    public final IMU imu;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    private final double inPerTick;

    public TwoDeadWheelLocalizer(HardwareMap hardwareMap, IMU imu, double inPerTick) {
        par = new RawEncoder(hardwareMap.get(DcMotorEx.class, "par"));
        perp = new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp"));
        this.imu = imu;

        lastParPos = par.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;
        lastHeading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        this.inPerTick = inPerTick;
    }

    public Twist2dDual<Time> update() {
        Encoder.PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        Encoder.PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();
        Rotation2d heading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        double headingVel = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - PAR_Y_TICKS * headingDelta,
                                parPosVel.velocity - PAR_Y_TICKS * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PERP_X_TICKS * headingDelta,
                                perpPosVel.velocity - PERP_X_TICKS * headingVel,
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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dIncrDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.RawEncoder;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    public static double PAR0_Y_TICKS = 0.0;
    public static double PAR1_Y_TICKS = 0.0;
    public static double PERP_X_TICKS = 0.0;

    public final Encoder par0, par1, perp;

    public final double inPerTick;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick) {
        par0 = new RawEncoder(hardwareMap.get(DcMotorEx.class, "par0"));
        par1 = new RawEncoder(hardwareMap.get(DcMotorEx.class, "par1"));
        perp = new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp"));

        lastPar0Pos = par0.getPositionAndVelocity().position;
        lastPar1Pos = par1.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;

        this.inPerTick = inPerTick;
    }

    public Twist2dIncrDual<Time> updateAndGetIncr() {
        Encoder.PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        Encoder.PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        Encoder.PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dIncrDual<Time> twistIncr = new Twist2dIncrDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PAR0_Y_TICKS * par1PosDelta - PAR1_Y_TICKS * par0PosDelta) / (PAR0_Y_TICKS - PAR1_Y_TICKS),
                                (PAR0_Y_TICKS * par1PosVel.velocity - PAR1_Y_TICKS * par0PosVel.velocity) / (PAR0_Y_TICKS - PAR1_Y_TICKS),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (PERP_X_TICKS / (PAR0_Y_TICKS - PAR1_Y_TICKS) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PERP_X_TICKS / (PAR0_Y_TICKS - PAR1_Y_TICKS) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PAR0_Y_TICKS - PAR1_Y_TICKS),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PAR0_Y_TICKS - PAR1_Y_TICKS),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        return twistIncr;
    }
}

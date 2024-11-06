package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
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
import org.firstinspires.ftc.teamcode.RoadRunner.messages.ThreeDeadWheelInputsMessage;

@Config
public class IMUPlusThreeDeadWheel implements Localizer {
        public static class Params {
            public double par0YTicks = -7448.968612548164;
            //public double par0YTicks = -2369.7230641368787; // y position of the first parallel encoder (in tick units)
            public double par1YTicks = 7719.537966877904;
            //public double par1YTicks = 2427.1598412821827; // y position of the second parallel encoder (in tick units)
            public double perpXTicks =  -11146.421923393827;
            //public double perpXTicks = -805.1190472263876; // x position of the perpendicular encoder (in tick units)
        }

        public static ThreeDeadWheelLocalizer.Params PARAMS = new ThreeDeadWheelLocalizer.Params();

        public final Encoder par0, par1, perp;

        public final double inPerTick;

        private int lastPar0Pos, lastPar1Pos, lastPerpPos;
        private boolean initialized;

        private final IMU imu;


        public IMUPlusThreeDeadWheel(HardwareMap hardwareMap, double inPerTick, IMU imu) {
            //TODO: Sort out if it is front left or back left for perp encoder
            //MOTORS: front right is par0 (C0), back right is par1 (C3), front left is perp (E0)
            par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "LFM"))); //left enc
            par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "LBM"))); //right enc
            perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "RBM")));

            perp.setDirection(DcMotorSimple.Direction.REVERSE);

            this.inPerTick = inPerTick;
            this.imu = imu;
            FlightRecorder.write("IMUPlusThreeDeadWheel", PARAMS);
        }

        public Twist2dDual<Time> update() {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
            PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
            PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
            PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

            FlightRecorder.write("IMUPlusThreeDeadWheel", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

            if (!initialized) {
                initialized = true;

                lastPar0Pos = par0PosVel.position;
                lastPar1Pos = par1PosVel.position;
                lastPerpPos = perpPosVel.position;

                return new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }

            int par0PosDelta = par0PosVel.position - lastPar0Pos;
            int par1PosDelta = par1PosVel.position - lastPar1Pos;
            int perpPosDelta = perpPosVel.position - lastPerpPos;

            Twist2dDual<Time> twist = new Twist2dDual<>(
                    new Vector2dDual<>(
                            new DualNum<Time>(new double[] {
                                    (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                    (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                            }).times(inPerTick),
                            new DualNum<Time>(new double[] {
                                    (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                    (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                            }).times(inPerTick)
                    ),
                    new DualNum<>(new double[] {
                            (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                            (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                    })
            );

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            return twist;
        }
    }



package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.RamseteController;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Twist2dIncrDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.BNO055Wrapper;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.LynxFirmwareVersion;
import org.firstinspires.ftc.teamcode.util.OverflowEncoder;
import org.firstinspires.ftc.teamcode.util.RawEncoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

@Config
public final class TankDrive {
    // drive model parameters
    public static double IN_PER_TICK = 0;
    public static double TRACK_WIDTH_TICKS = 0;

    // feedforward parameters
    public static double kS = 0;
    public static double kV = 0;
    public static double kA = 0;

    // path profile parameters
    public static double MAX_WHEEL_VEL = 50;
    public static double MIN_PROFILE_ACCEL = -30;
    public static double MAX_PROFILE_ACCEL = 50;

    // turn profile parameters
    public static double MAX_ANG_VEL = Math.PI; // shared with path
    public static double MAX_ANG_ACCEL = Math.PI;

    // path controller gains
    public static double RAMSETE_ZETA = 0.7; // in the range (0, 1)
    public static double RAMSETE_BBAR = 2.0; // positive

    // turn controller gains
    public static double TURN_GAIN = 0.0;
    public static double TURN_VEL_GAIN = 0.0;

    public final TankKinematics kinematics = new TankKinematics(IN_PER_TICK * TRACK_WIDTH_TICKS);

    public final MotorFeedforward feedforward = new MotorFeedforward(kS, kV, kA);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            MAX_ANG_VEL, -MAX_ANG_ACCEL, MAX_ANG_ACCEL);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(MAX_WHEEL_VEL),
                    new AngularVelConstraint(MAX_ANG_VEL)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(MIN_PROFILE_ACCEL, MAX_PROFILE_ACCEL);

    public final List<DcMotorEx> leftMotors, rightMotors;

    public final BNO055Wrapper imu;

    public final VoltageSensor voltageSensor;

    public final Localizer localizer;
    public Pose2d pose;

    public final double inPerTick = IN_PER_TICK;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public class DriveLocalizer implements Localizer {
        public final List<Encoder> leftEncs, rightEncs;

        private double lastLeftPos, lastRightPos;

        public DriveLocalizer() {
            {
                List<Encoder> leftEncs = new ArrayList<>();
                for (DcMotorEx m : leftMotors) {
                    Encoder e = new OverflowEncoder(new RawEncoder(m));
                    leftEncs.add(e);
                    lastLeftPos += e.getPositionAndVelocity().position;
                }
                lastLeftPos /= leftEncs.size();
                this.leftEncs = Collections.unmodifiableList(leftEncs);
            }

            {
                List<Encoder> rightEncs = new ArrayList<>();
                for (DcMotorEx m : rightMotors) {
                    Encoder e = new OverflowEncoder(new RawEncoder(m));
                    rightEncs.add(e);
                    lastRightPos += e.getPositionAndVelocity().position;
                }
                lastRightPos /= rightEncs.size();
                this.rightEncs = Collections.unmodifiableList(rightEncs);
            }
        }

        @Override
        public Twist2dIncrDual<Time> updateAndGetIncr() {
            double meanLeftPos = 0.0, meanLeftVel = 0.0;
            for (Encoder e : leftEncs) {
                Encoder.PositionVelocityPair p = e.getPositionAndVelocity();
                meanLeftPos += p.position;
                meanLeftVel += p.velocity;
            }
            meanLeftPos /= leftEncs.size();
            meanLeftVel /= leftEncs.size();

            double meanRightPos = 0.0, meanRightVel = 0.0;
            for (Encoder e : rightEncs) {
                Encoder.PositionVelocityPair p = e.getPositionAndVelocity();
                meanRightPos += p.position;
                meanRightVel += p.velocity;
            }
            meanRightPos /= rightEncs.size();
            meanRightVel /= rightEncs.size();

            TankKinematics.WheelIncrements<Time> incrs = new TankKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[] {
                            meanLeftPos - lastLeftPos,
                            meanLeftVel
                    }).times(inPerTick),
                    new DualNum<Time>(new double[] {
                            meanRightPos - lastRightPos,
                            meanRightVel,
                    }).times(inPerTick)
            );

            lastLeftPos = meanLeftPos;
            lastRightPos = meanRightPos;

            return kinematics.forward(incrs);
        }
    }

    public TankDrive(HardwareMap hardwareMap, Pose2d pose) {
        this.pose = pose;

        LynxFirmwareVersion.throwIfAnyModulesBelowVersion(hardwareMap,
                new LynxFirmwareVersion(1, 8, 2));

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "left"));
        rightMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "right"));

        for (DcMotorEx m : leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotorEx m : rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        BNO055IMU baseImu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        baseImu.initialize(parameters);
        imu = new BNO055Wrapper(baseImu);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new TankDrive.DriveLocalizer();
    }

    public void setDrivePowers(Twist2d powers) {
        TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(Twist2dDual.constant(powers, 1));
        for (DcMotorEx m : leftMotors) {
            m.setPower(wheelVels.left.get(0));
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(wheelVels.right.get(0));
        }
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    (int) Math.ceil(t.path.length() / 2));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.trans.x;
                yPoints[i] = p.trans.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                for (DcMotorEx m : leftMotors) {
                    m.setPower(0);
                }
                for (DcMotorEx m : rightMotors) {
                    m.setPower(0);
                }

                return false;
            }

            DualNum<Time> x = timeTrajectory.profile.get(t);

            Pose2dDual<Arclength> txWorldTarget = timeTrajectory.path.get(x.value(), 3);

            updatePoseEstimateAndGetActualVel();

            Twist2dDual<Time> command = new RamseteController(kinematics.trackWidth, RAMSETE_ZETA, RAMSETE_BBAR)
                    .compute(x, txWorldTarget, pose);

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            for (DcMotorEx m : leftMotors) {
                m.setPower(feedforward.compute(wheelVels.left) / voltage);
            }
            for (DcMotorEx m : rightMotors) {
                m.setPower(feedforward.compute(wheelVels.right) / voltage);
            }

            p.put("x", pose.trans.x);
            p.put("y", pose.trans.y);
            p.put("heading (deg)", Math.toDegrees(pose.rot.log()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.trans.x);
            p.put("yError", error.trans.y);
            p.put("headingError (deg)", Math.toDegrees(error.rot.log()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                for (DcMotorEx m : leftMotors) {
                    m.setPower(0);
                }
                for (DcMotorEx m : rightMotors) {
                    m.setPower(0);
                }

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);

            Twist2d robotVelRobot = updatePoseEstimateAndGetActualVel();

            Twist2dDual<Time> command = new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0, 0), 3),
                    txWorldTarget.rot.velocity().plus(
                            TURN_GAIN * pose.rot.minus(txWorldTarget.rot.value()) +
                            TURN_VEL_GAIN * (robotVelRobot.rotVel - txWorldTarget.rot.velocity().value())
                    )
            );

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            for (DcMotorEx m : leftMotors) {
                m.setPower(feedforward.compute(wheelVels.left) / voltage);
            }
            for (DcMotorEx m : rightMotors) {
                m.setPower(feedforward.compute(wheelVels.right) / voltage);
            }

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.trans.x, turn.beginPose.trans.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.trans.x, turn.beginPose.trans.y, 2);
        }
    }

    public Twist2d updatePoseEstimateAndGetActualVel() {
        Twist2dIncrDual<Time> incr = localizer.updateAndGetIncr();
        pose = pose.plus(incr.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        return incr.velocity().value();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.trans.x;
            yPoints[i] = t.trans.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    private static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.trans.x, t.trans.y, ROBOT_RADIUS);

        Vector2d halfv = t.rot.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.trans.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                beginPose, 1e-6,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint,
                0.25
        );
    }
}

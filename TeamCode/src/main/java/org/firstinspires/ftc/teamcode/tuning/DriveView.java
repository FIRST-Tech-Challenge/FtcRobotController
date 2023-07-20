package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.MidpointTimer;
import org.firstinspires.ftc.teamcode.util.OverflowEncoder;
import org.firstinspires.ftc.teamcode.util.RawEncoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

final class DriveView {
    public final String type;

    public final double inPerTick;
    public final double maxVel, minAccel, maxAccel;

    public final List<LynxModule> lynxModules;

    public final List<DcMotorEx> leftMotors, rightMotors;
    public final List<DcMotorEx> motors;

    // invariant: (leftEncs.isEmpty() && rightEncs.isEmpty()) ||
    //                  (parEncs.isEmpty() && perpEncs.isEmpty())
    public final List<RawEncoder> leftEncs, rightEncs, parEncs, perpEncs;
    public final List<RawEncoder> forwardEncs;

    public final List<Encoder> forwardEncsWrapped;

    public final IMU imu;

    public final VoltageSensor voltageSensor;

    private final MecanumDrive md;
    private final TankDrive td;

    private static RawEncoder unwrap(Encoder e) {
        if (e instanceof OverflowEncoder) {
            return ((OverflowEncoder) e).encoder;
        } else {
            return (RawEncoder) e;
        }
    }

    public DriveView(HardwareMap hardwareMap) {
        lynxModules = hardwareMap.getAll(LynxModule.class);

        final Localizer localizer;
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            type = "mecanum";

            inPerTick = MecanumDrive.IN_PER_TICK;
            maxVel = MecanumDrive.MAX_WHEEL_VEL;
            minAccel = MecanumDrive.MIN_PROFILE_ACCEL;
            maxAccel = MecanumDrive.MAX_PROFILE_ACCEL;

            md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)); td = null;
            leftMotors = Arrays.asList(md.leftFront, md.leftBack);
            rightMotors = Arrays.asList(md.rightFront, md.rightBack);
            imu = md.imu;
            voltageSensor = md.voltageSensor;

            localizer = md.localizer;
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            type = "tank";

            inPerTick = TankDrive.IN_PER_TICK;
            maxVel = TankDrive.MAX_WHEEL_VEL;
            minAccel = TankDrive.MIN_PROFILE_ACCEL;
            maxAccel = TankDrive.MAX_PROFILE_ACCEL;

            td = new TankDrive(hardwareMap, new Pose2d(0, 0, 0)); md = null;
            leftMotors = td.leftMotors;
            rightMotors = td.rightMotors;
            imu = td.imu;
            voltageSensor = td.voltageSensor;

            localizer = td.localizer;
        } else {
            throw new AssertionError();
        }

        forwardEncsWrapped = new ArrayList<>();

        if (localizer instanceof TwoDeadWheelLocalizer) {
            TwoDeadWheelLocalizer l2 = (TwoDeadWheelLocalizer) localizer;
            parEncs = Collections.singletonList(unwrap(l2.par));
            perpEncs = Collections.singletonList(unwrap(l2.perp));
            leftEncs = Collections.emptyList();
            rightEncs = Collections.emptyList();

            forwardEncsWrapped.add(l2.par);
            forwardEncsWrapped.add(l2.perp);
        } else if (localizer instanceof ThreeDeadWheelLocalizer) {
            ThreeDeadWheelLocalizer l3 = (ThreeDeadWheelLocalizer) localizer;
            parEncs = Arrays.asList(unwrap(l3.par0), unwrap(l3.par1));
            perpEncs = Collections.singletonList(unwrap(l3.perp));
            leftEncs = Collections.emptyList();
            rightEncs = Collections.emptyList();

            forwardEncsWrapped.add(l3.par0);
            forwardEncsWrapped.add(l3.par1);
            forwardEncsWrapped.add(l3.perp);
        } else if (localizer instanceof MecanumDrive.DriveLocalizer) {
            MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) localizer;
            parEncs = Collections.emptyList();
            perpEncs = Collections.emptyList();
            leftEncs = Arrays.asList(unwrap(dl.leftFront), unwrap(dl.leftRear));
            rightEncs = Arrays.asList(unwrap(dl.rightFront), unwrap(dl.rightRear));

            forwardEncsWrapped.add(dl.leftFront);
            forwardEncsWrapped.add(dl.leftRear);
            forwardEncsWrapped.add(dl.rightFront);
            forwardEncsWrapped.add(dl.rightRear);
        } else if (localizer instanceof TankDrive.DriveLocalizer) {
            TankDrive.DriveLocalizer dl = (TankDrive.DriveLocalizer) localizer;
            parEncs = Collections.emptyList();
            perpEncs = Collections.emptyList();
            leftEncs = new ArrayList<>();
            for (Encoder e : dl.leftEncs) {
                leftEncs.add(unwrap(e));

                forwardEncsWrapped.add(e);
            }
            rightEncs = new ArrayList<>();
            for (Encoder e : dl.rightEncs) {
                rightEncs.add(unwrap(e));

                forwardEncsWrapped.add(e);
            }
        } else {
            throw new AssertionError();
        }

        motors = new ArrayList<>();
        motors.addAll(leftMotors);
        motors.addAll(rightMotors);

        forwardEncs = new ArrayList<>();
        forwardEncs.addAll(leftEncs);
        forwardEncs.addAll(rightEncs);
        forwardEncs.addAll(parEncs);

        List<RawEncoder> allEncs = new ArrayList<>();
        allEncs.addAll(forwardEncs);
        allEncs.addAll(perpEncs);
    }

    public MotorFeedforward feedforward() {
        if (md != null) {
            return new MotorFeedforward(MecanumDrive.kS, MecanumDrive.kV, MecanumDrive.kA);
        }

        if (td != null) {
            return new MotorFeedforward(TankDrive.kS, TankDrive.kV, TankDrive.kA);
        }

        throw new AssertionError();
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        if (md != null) {
            md.setDrivePowers(powers);
            return;
        }

        if (td != null) {
            td.setDrivePowers(powers);
            return;
        }

        throw new AssertionError();
    }

    public void setBulkCachingMode(LynxModule.BulkCachingMode mode) {
        for (LynxModule m : lynxModules) {
            m.setBulkCachingMode(mode);
        }
    }

    public Map<SerialNumber, Double> resetAndBulkRead(MidpointTimer t) {
        final Map<SerialNumber, Double> times = new HashMap<>();
        for (LynxModule m : lynxModules) {
            m.clearBulkCache();

            t.addSplit();
            m.getBulkData();
            times.put(m.getSerialNumber(), t.addSplit());
        }
        return times;
    }
}

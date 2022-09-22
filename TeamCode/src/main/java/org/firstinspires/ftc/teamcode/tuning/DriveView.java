package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.BNO055Wrapper;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.OverflowEncoder;
import org.firstinspires.ftc.teamcode.util.RawEncoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

final class DriveView {
    public final String type;

    public final double inPerTick;
    public final double maxVel, minAccel, maxAccel;

    public final List<DcMotorEx> leftMotors, rightMotors;
    public final List<DcMotorEx> motors;

    // invariant: (leftEncs.isEmpty() && rightEncs.isEmpty()) ||
    //                  (parEncs.isEmpty() && perpEncs.isEmpty())
    public final List<RawEncoder> leftEncs, rightEncs, parEncs, perpEncs;
    public final List<RawEncoder> forwardEncs;

    public final BNO055Wrapper imu;

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

        if (localizer instanceof TwoDeadWheelLocalizer) {
            TwoDeadWheelLocalizer l2 = (TwoDeadWheelLocalizer) localizer;
            parEncs = Collections.singletonList(unwrap(l2.par));
            perpEncs = Collections.singletonList(unwrap(l2.perp));
            leftEncs = Collections.emptyList();
            rightEncs = Collections.emptyList();
        } else if (localizer instanceof ThreeDeadWheelLocalizer) {
            ThreeDeadWheelLocalizer l3 = (ThreeDeadWheelLocalizer) localizer;
            parEncs = Arrays.asList(unwrap(l3.par0), unwrap(l3.par1));
            perpEncs = Collections.singletonList(unwrap(l3.perp));
            leftEncs = Collections.emptyList();
            rightEncs = Collections.emptyList();
        } else if (localizer instanceof MecanumDrive.DriveLocalizer) {
            MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) localizer;
            parEncs = Collections.emptyList();
            perpEncs = Collections.emptyList();
            leftEncs = Arrays.asList(unwrap(dl.leftFront), unwrap(dl.leftRear));
            rightEncs = Arrays.asList(unwrap(dl.rightFront), unwrap(dl.rightRear));
        } else if (localizer instanceof TankDrive.DriveLocalizer) {
            TankDrive.DriveLocalizer dl = (TankDrive.DriveLocalizer) localizer;
            parEncs = Collections.emptyList();
            perpEncs = Collections.emptyList();
            leftEncs = new ArrayList<>();
            for (Encoder e : dl.leftEncs) {
                leftEncs.add(unwrap(e));
            }
            rightEncs = new ArrayList<>();
            for (Encoder e : dl.rightEncs) {
                rightEncs.add(unwrap(e));
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

        DcMotorController c1 = allEncs.get(0).getController();
        for (Encoder e : allEncs) {
            DcMotorController c2 = e.getController();
            if (c1 != c2) {
                throw new IllegalArgumentException("all encoders must be attached to the same hub");
            }
        }
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

    public void setDrivePowers(Twist2d powers) {
        if (md != null) {
            md.setDrivePowers(powers);
        }

        if (td != null) {
            td.setDrivePowers(powers);
        }

        throw new AssertionError();
    }
}

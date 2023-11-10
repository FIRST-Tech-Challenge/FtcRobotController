package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.AngularRampLogger;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.LateralPushTest;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class TuningOpModes {
    public static final Class<?> DRIVE_CLASS = MecanumDrive.class;

    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = false;

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        DriveViewFactory dvf;
        dvf = hardwareMap -> {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//            MecanumDrive drive = new MecanumDrive(hardwareMap, shared.motorLeftFront, shared.motorLeftBack, shared.motorRightBack, shared.motorRightFront new Pose2d(0, 0, 0));

            List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
            List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
            if (drive.localizer instanceof MecanumDrive.DriveLocalizer) {
                MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) drive.localizer;
                leftEncs.add(dl.leftFront);
                leftEncs.add(dl.leftRear);
                rightEncs.add(dl.rightFront);
                rightEncs.add(dl.rightRear);
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) drive.localizer;
                parEncs.add(dl.par0);
                parEncs.add(dl.par1);
                perpEncs.add(dl.perp);
            } else if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                TwoDeadWheelLocalizer dl = (TwoDeadWheelLocalizer) drive.localizer;
                parEncs.add(dl.par);
                perpEncs.add(dl.perp);
            } else {
                throw new IllegalArgumentException("unknown localizer: " + drive.localizer.getClass().getName());
            }

            return new DriveView(
                DriveType.MECANUM,
                    MecanumDrive.PARAMS.inPerTick,
                    MecanumDrive.PARAMS.maxWheelVel,
                    MecanumDrive.PARAMS.minProfileAccel,
                    MecanumDrive.PARAMS.maxProfileAccel,
                    hardwareMap.getAll(LynxModule.class),
                    Arrays.asList(
                            drive.leftFront,
                            drive.leftBack
                    ),
                    Arrays.asList(
                            drive.rightFront,
                            drive.rightBack
                    ),
                    leftEncs,
                    rightEncs,
                    parEncs,
                    perpEncs,
                    drive.imu,
                    drive.voltageSensor,
                    drive.feedforward
            );
        };

        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));

        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);
    }
}

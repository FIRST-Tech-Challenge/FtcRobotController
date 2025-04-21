package org.firstinspires.ftc.team00000.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.team00000.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team00000.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.team00000.roadrunner.TwoDeadWheelLocalizer;

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
        if (DRIVE_CLASS.equals(MecanumDrive.class)) {
            dvf = hardwareMap -> {
                MecanumDrive md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                if (md.localizer instanceof MecanumDrive.DriveLocalizer) {
                    MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) md.localizer;
                    leftEncs.add(dl.leftFront);
                    leftEncs.add(dl.leftBack);
                    rightEncs.add(dl.rightFront);
                    rightEncs.add(dl.rightBack);
                } else if (md.localizer instanceof ThreeDeadWheelLocalizer) {
                    ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) md.localizer;
                    parEncs.add(dl.par0);
                    parEncs.add(dl.par1);
                    perpEncs.add(dl.perp);
                } else if (md.localizer instanceof TwoDeadWheelLocalizer) {
                    TwoDeadWheelLocalizer dl = (TwoDeadWheelLocalizer) md.localizer;
                    parEncs.add(dl.par);
                    perpEncs.add(dl.perp);
                } else {
                    throw new RuntimeException("unknown localizer: " + md.localizer.getClass().getName());
                }

                return new DriveView(
                        DriveType.MECANUM,
                        MecanumDrive.PARAMS.inPerTick,
                        MecanumDrive.PARAMS.maxWheelVel,
                        MecanumDrive.PARAMS.minProfileAccel,
                        MecanumDrive.PARAMS.maxProfileAccel,
                        hardwareMap.getAll(LynxModule.class),
                        Arrays.asList(
                                md.leftFront,
                                md.leftBack
                        ),
                        Arrays.asList(
                                md.rightFront,
                                md.rightBack
                        ),
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        md.lazyImu,
                        md.voltageSensor,
                        () -> new MotorFeedforward(MecanumDrive.PARAMS.kS,
                                MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                                MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick)
                );
            };
        } else {
            throw new RuntimeException();
        }

        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class), new DeadWheelDirectionDebugger(dvf));

        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}

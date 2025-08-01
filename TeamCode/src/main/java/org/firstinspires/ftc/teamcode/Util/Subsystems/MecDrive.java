package org.firstinspires.ftc.teamcode.Util.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.DashboardPoseTracker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


import java.lang.annotation.*;
import java.util.concurrent.atomic.AtomicLong;

public class MecDrive implements Subsystem {
    public static final MecDrive INSTANCE = new MecDrive();
    public static Follower follower;
    public static boolean isSlowed = false;
    public static double slowSpeed = 0.25;

    public static DcMotorEx fl;
    public static DcMotorEx fr;
    public static DcMotorEx bl;
    public static DcMotorEx br;
    public static Telemetry telemetry;
    public static DashboardPoseTracker dashboardPoseTracker;
    public MecDrive() {}

    public static void speedSlow(){
        isSlowed = true;
    }

    public static void speedFast(){
        isSlowed = false;
    }

    public static void drive(double x, double y, double z) {
        follower.setTeleOpMovementVectors(
                x * (isSlowed ? slowSpeed : 1),
                y * (isSlowed ? slowSpeed : 1),
                z * (isSlowed ? slowSpeed : 1),
                false
        );
        follower.update();
    }


    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        telemetry = opMode.getOpMode().telemetry;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new Follower(FeatureRegistrar.getActiveOpMode().hardwareMap, FConstants.class, LConstants.class);
        dashboardPoseTracker = MecDrive.follower.getDashboardPoseTracker();

        HardwareMap hMap = opMode.getOpMode().hardwareMap;
        fl = hMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        bl = hMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        fr = hMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);
        br = hMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);

        fl.setDirection(FollowerConstants.leftFrontMotorDirection);
        bl.setDirection(FollowerConstants.leftRearMotorDirection);
        fr.setDirection(FollowerConstants.rightFrontMotorDirection);
        br.setDirection(FollowerConstants.rightRearMotorDirection);


        follower.setStartingPose(new Pose(0, 0, 0));
        setDefaultCommand(drive(Mercurial.gamepad1()));


    }




    @Override
    public void preUserStartHook(@NonNull Wrapper opMode) {
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {}

    public static Lambda drive(BoundGamepad gamepad){
        return new Lambda("drive")
                .addRequirements(INSTANCE)
                .setExecute(() -> drive(
                        -gamepad.leftStickY().state(),
                        gamepad.leftStickX().state(),
                        gamepad.rightStickX().state()
                ))
                .setFinish(() -> false);
    }
    public static Lambda push(double pow, long time){
        AtomicLong startTime = new AtomicLong();
        return new Lambda("push-drive")
                .setInit(() -> startTime.set(System.currentTimeMillis()))
                .setExecute(
                        () -> {
                            fl.setPower(pow);
                            fr.setPower(pow);
                            bl.setPower(pow);
                            br.setPower(pow);
                        }
                )
                .setFinish(() -> System.currentTimeMillis() - startTime.get() > time);
    }

    public static Lambda toggleSlow() {
        return new Lambda("toggle-slow")
                .setInit(() -> isSlowed = !isSlowed)
                .setFinish(() -> true);
    }

    public static Lambda slow(){
        return new Lambda("slow")
                .setInit(MecDrive::speedSlow);
    }

    public static Lambda fast(){
        return new Lambda("fast")
                .setInit(MecDrive::speedFast);
    }

    public static Lambda followPath(Path path) {
        return new Lambda("follow-path")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.followPath(path, true))
                .setExecute(() -> {
                    follower.update();
                    follower.telemetryDebug(telemetry);

                })
                .setFinish(() -> !follower.isBusy())
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }

    public static Lambda followPath(Path path, boolean hold) {
        return new Lambda("follow-path")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.followPath(path, hold))
                .setExecute(() -> {
                    follower.update();
                    follower.telemetryDebug(telemetry);
                })
                .setFinish(() -> !follower.isBusy())
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }

    public static Lambda followPathChain(PathChain chain) {
        return new Lambda("follow-path-chain")
                .addRequirements(INSTANCE)
                .setInit(() -> follower.followPath(chain, true))
                .setExecute(() -> {
                    follower.update();
                    telemetry.addData("pinpoint cooked", follower.isLocalizationNAN());
                })
                .setFinish(() -> !follower.isBusy() || follower.isRobotStuck());
    }

    public static Lambda turnToCommand(double degrees) {
        return new Lambda("turn-to-command")
                .addRequirements(INSTANCE)
                .setInit(() -> follower.turnToDegrees(degrees))
                .setExecute(() -> {
                    follower.update();
                    telemetry.addData("pinpoint cooked", follower.isLocalizationNAN());
                })
                .setFinish(() -> !follower.isBusy() || follower.isRobotStuck());
    }

    public static Lambda turnByCommand(double degrees, boolean isLeft) {
        return new Lambda("turn-by-command")
                .addRequirements(INSTANCE)
                .setInit(() -> follower.turnDegrees(degrees, isLeft))
                .setExecute(() -> {
                    follower.update();
                    telemetry.addData("pinpoint cooked", follower.isLocalizationNAN());
                })
                .setFinish(() -> !follower.isBusy() || follower.isRobotStuck());
    }



    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }

}
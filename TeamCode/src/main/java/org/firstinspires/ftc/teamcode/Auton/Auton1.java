package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name = "Auton1")
public class Auton1 extends OpMode {

    private Follower follower;

    private final PoseFactory p = PoseFactory.degrees();

    private final ElapsedTime shootTime = new ElapsedTime();

    private DcMotor outtake;
    private DcMotor intake;

    //no need for servos because we are not intaking anything.

    private DcMotor transfer;


    private final Pose startPose = p.of(12, 12, 90);

    private final Pose endPose = p.of(60, 12, 90);


    public Path path1() {
        return line(startPose, endPose).constant(startPose);
    }


    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

    }


    @Override
    public void start() {
        follower.follow(path1());
        schedule(shoot());
    }


    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

    }


    @Override
    public void stop() {
        Scheduler.reset();
        if (outtake != null) outtake.setPower(0);
        if (intake != null) intake.setPower(0);
        if (transfer != null) transfer.setPower(0);
    }

    public Command shoot() {
        return Command.build()
                .setStart(() -> {
                    shootTime.reset();
                    outtake.setPower(0.85);
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .setDone(() -> shootTime.milliseconds() >= 10000)
                .setEnd(interrupted -> {
                    outtake.setPower(0);
                    intake.setPower(0);
                    transfer.setPower(0);
                });
    }
}

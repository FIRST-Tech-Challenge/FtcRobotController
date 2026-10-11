package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.qualcomm.robotcore.util.ElapsedTime;



public class Auton1 extends OpMode {

    private Follower follower;

    private final PoseFactory p = PoseFactory.degrees();

    private ElapsedTime shootTime = new ElapsedTime();

    private DcMotor outtake = hardwareMap.get(DcMotor.class, "outtake");
    private DcMotor intake = hardwareMap.get(DcMotor.class, "intake");


    private final Pose startPose = p.of(82, 12, 90);

    private final Pose endPose = p.of(82, 24, 90);


    public Path path1() {
        return line(startPose, endPose).constant(startPose);
    }


    @Override
    public void init() {
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    @Override
    public void loop() {

    }

    /*public Command shoot() {
        return Command.build()
                .setStart(
            shootTime.reset();
            outtake.setPower(1);
            intake.setPower(1);
        )
        if (shootTime.milliseconds() == 5) {
            outtake.setPower(0);
            intake.setPower(0);
        }
    }*/
}



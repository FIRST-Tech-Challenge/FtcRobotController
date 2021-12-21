package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drive.roadrunner.RunCommand;
import org.firstinspires.ftc.teamcode.commands.drive.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.drive.roadrunner.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.leds.blinkin.ShowAllianceColor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateArm;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateBC4HDrive;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateCarousel;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateIntake;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateLEDs;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateWebCam;
import org.firstinspires.ftc.teamcode.subsystems.drive.roadrunner.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;



@Autonomous(name="Auto: FTCLib RR  Sample1", group="FTCLib")
public class FTCLibRRDriveSample1 extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private TrajectoryFollowerCommand sample1Follower1;
    private TrajectoryFollowerCommand sample1Follower2;
    private TrajectoryFollowerCommand sample1Follower3;
    private TrajectoryFollowerCommand sample1Follower4;
    private TrajectoryFollowerCommand sample1Follower5;
    private TrajectoryFollowerCommand sample1Follower6;

    private TurnCommand turnCommand1;
    private TurnCommand turnCommand2;
    private TurnCommand turnCommand3;
    private TurnCommand turnCommand4;
    private TurnCommand turnCommand5;

    private Command trajGroup;
    private Pose2d startPose;

    @Override
    public void initialize() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setAutoClear(false);

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        startPose = new Pose2d(-37, 72, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        LEDSubsystem ledSubsystem = new LEDSubsystem(hardwareMap,"blinkin");
        ShowAllianceColor allianceColor = new ShowAllianceColor(ledSubsystem,ShowAllianceColor.AllianceColor.BLUE);

        /*.splineTo(new Vector2d(-15,40),Math.toRadians(270))
                .addDisplacementMarker(()->{}) //step 3
                .waitSeconds(3) //step 3

                .turn(Math.toRadians(-135)) //step 4

                .lineTo(new Vector2d(-46,55))//step 5
                .addDisplacementMarker(()->{}) //step 6
                .waitSeconds(3)

                .turn(Math.toRadians(135)) //step 7

                .splineToConstantHeading(new Vector2d(0,64),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(42,64),Math.toRadians(0))

                .turn(Math.toRadians(90))

                .addDisplacementMarker(()->{}) //step 10
                .waitSeconds(1) //step

                .splineToSplineHeading(new Pose2d(8,64,Math.toRadians(270)),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-15,40),Math.toRadians(57))
                .turn(Math.toRadians(90))
                .back(45)*/

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-15,40),Math.toRadians(270))
                .addDisplacementMarker(()-> {
                    telemetry.addData("Path 1", "performing path 1 action");
                })
                .build();

        turnCommand1 = new TurnCommand(drive, Math.toRadians(-135));

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(-135))))
                .lineTo(new Vector2d(-46,55))//step 5
                .addDisplacementMarker(()->{
                    telemetry.addData("Path 2", "performing path 2 action");
                }) //step 6
                .build();

        turnCommand2 = new TurnCommand(drive, Math.toRadians(135));

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end().plus(new Pose2d(0, 0, Math.toRadians(135))))
                .splineToConstantHeading(new Vector2d(0,64),Math.toRadians(0)) //step 8
                .splineToConstantHeading(new Vector2d(42,64),Math.toRadians(0))
                .build();

        turnCommand3 = new TurnCommand(drive, Math.toRadians(90));

        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(()->{
                    telemetry.addData("Path 4", "performing path 4 action");
                    allianceColor.schedule();
                }) //step 10
                .build();

        //turnCommand4 = new TurnCommand(drive, 90);

        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(8,64,Math.toRadians(270)),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-15,40),Math.toRadians(57))
                .build();

        turnCommand5 = new TurnCommand(drive, -90);

        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d())
                .back(45)
                .build();

        sample1Follower1 = new TrajectoryFollowerCommand(drive,traj1);
        sample1Follower2 = new TrajectoryFollowerCommand(drive,traj2);
        sample1Follower3 = new TrajectoryFollowerCommand(drive,traj3);
        sample1Follower4 = new TrajectoryFollowerCommand(drive,traj4);
        sample1Follower5 = new TrajectoryFollowerCommand(drive,traj5);
        sample1Follower6 = new TrajectoryFollowerCommand(drive,traj6);

        schedule(new WaitUntilCommand(this::isStarted).andThen(
                sample1Follower1.andThen(turnCommand1,
                        sample1Follower2,turnCommand2,
                        sample1Follower3,turnCommand3,
                        sample1Follower4,sample1Follower5,
                        turnCommand5, sample1Follower6)
        ));

        telemetry.update();
    }
}

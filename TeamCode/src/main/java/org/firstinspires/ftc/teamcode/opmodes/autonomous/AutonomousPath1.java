package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.commands.carousel.MoveCarousel;
import org.firstinspires.ftc.teamcode.commands.carousel.MoveCarouselToPosition;
import org.firstinspires.ftc.teamcode.commands.carousel.SpinOneDuckCarousel;
import org.firstinspires.ftc.teamcode.commands.carousel.StopCarousel;
import org.firstinspires.ftc.teamcode.commands.drive.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.drive.roadrunner.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.leds.blinkin.ShowAllianceColor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.BluePath1;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateCarousel;
import org.firstinspires.ftc.teamcode.subsystems.carousel.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.roadrunner.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;

import java.util.function.BooleanSupplier;


@Autonomous(name="AutonomousPath1", group="FTCLib")
public class AutonomousPath1 extends CommandOpMode {

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

        schedule(new InstantCommand(() -> {
            telemetry.clearAll();
            telemetry.addLine("What is your Alliance?");
            telemetry.addLine("Press (X) for BLUE, (Y) for RED");
            telemetry.update();
        }));

        GamepadEx settingsOp = new GamepadEx(gamepad1);

        //X (Blue) button
        Button blueAlliance = new GamepadButton(settingsOp, GamepadKeys.Button.X);
        //Y (Red) button
        Button redAlliance = new GamepadButton(settingsOp, GamepadKeys.Button.B);

        Button path1Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_UP);
        Button path2Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_RIGHT);
        Button path3Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_DOWN);
        Button path4Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_LEFT);


        blueAlliance.whenPressed(()->{
            Alliance.getInstance().setAllicanceTeam(Alliance.AllianceTeam.BLUE);

        });
        redAlliance.whenPressed(()->{
            Alliance.getInstance().setAllicanceTeam(Alliance.AllianceTeam.RED);
        });

        path1Selector.whenPressed(()->{
            if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE){
                BluePath1 bluePath1 = new BluePath1(hardwareMap,telemetry);
                bluePath1.createPath();
                bluePath1.execute(this);
            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED){

            }
        });




        /*drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);

        startPose = new Pose2d(-36, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        LEDSubsystem ledSubsystem = new LEDSubsystem(hardwareMap,"blinkin");
        ShowAllianceColor allianceColor = new ShowAllianceColor(ledSubsystem,ShowAllianceColor.AllianceColor.BLUE);

        CarouselSubsystem carouselSubsystem = new CarouselSubsystem(hardwareMap, "carousel");

        /*
        carouselSubsystem.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);*/

        /*//Option 2 - How Alex would do it with what we currently have
        MoveCarouselToPosition moveCarouselToPosition = new MoveCarouselToPosition(carouselSubsystem,1000,0.3,telemetry);
        StopCarousel stopCarousel = new StopCarousel(carouselSubsystem,telemetry);
        BooleanSupplier carouselPosition = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return carouselSubsystem.getCarouselCurrentPosition() >= 1000;
            }
        };*/

        /*//Option 3 - How Alex would do it with what we currently have
        CreateCarousel createCarousel = new CreateCarousel(hardwareMap,"carousel",telemetry);
        createCarousel.createAuto();
        SequentialCommandGroup carouselGroup = new SequentialCommandGroup(createCarousel.getMoveCarouselToPosition(),
                new WaitUntilCommand(createCarousel.hasMaxEncoderCountSupplier()).andThen(createCarousel.getStopCarousel()));*/

        /*//Option 2 - How Alex would do it with what we currently have
        SequentialCommandGroup opt2CarouselGroup = new SequentialCommandGroup(moveCarouselToPosition,
                new WaitUntilCommand(carouselPosition).andThen(stopCarousel));

         */

/*
        SpinOneDuckCarousel spinOneDuckCarousel = new SpinOneDuckCarousel(carouselSubsystem,0.3);


        Command spinDuck = new SequentialCommandGroup(
            spinOneDuckCarousel, new WaitUntilCommand( spinOneDuckCarousel::isFinished )
        );

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
/*
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-60, 60))
                .addDisplacementMarker(()-> {
                    telemetry.addData("Path 1", "performing path 1 action");
                    allianceColor.schedule();
                    //spinDuck.schedule();
                    //spinOneDuckCarousel.schedule();


                    /*Option 2 How Alex would do it with what we currently have
                    SequentialCommandGroup carouselGroup = new SequentialCommandGroup(moveCarousel,
                            new WaitUntilCommand(carouselPosition).andThen(stopCarousel));
                     */

                     //Option 3 How Alex would do it with what we currently have

                        //carouselGroup.schedule();

/*
                })
                .build();*/

        //turnCommand1 = new TurnCommand(drive, Math.toRadians(-135));

        /*Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-60, 24))
                .addDisplacementMarker(()->{
                    telemetry.addData("Path 2", "performing path 2 action");
                }) //step 6
                .build();*/

        //turnCommand2 = new TurnCommand(drive, Math.toRadians(135));

        /*Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToLinearHeading(new Pose2d(-32, 24, Math.toRadians(0)),Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-60, 36, Math.toRadians(270)),Math.toRadians(270))
                .build();

        //turnCommand3 = new TurnCommand(drive, Math.toRadians(90));

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

        //turnCommand5 = new TurnCommand(drive, -90);

        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d())
                .back(45)
                .build();*/

        /*sample1Follower1 = new TrajectoryFollowerCommand(drive,traj1);
        sample1Follower2 = new TrajectoryFollowerCommand(drive,traj2);
        //sample1Follower3 = new TrajectoryFollowerCommand(drive,traj3);
        //sample1Follower4 = new TrajectoryFollowerCommand(drive,traj4);
        //sample1Follower5 = new TrajectoryFollowerCommand(drive,traj5);
        //sample1Follower6 = new TrajectoryFollowerCommand(drive,traj6);

        schedule(new WaitUntilCommand(this::isStarted).andThen(
                sample1Follower1.andThen(carouselGroup,
                        sample1Follower2)
        ));

        telemetry.update();*/
    }
}

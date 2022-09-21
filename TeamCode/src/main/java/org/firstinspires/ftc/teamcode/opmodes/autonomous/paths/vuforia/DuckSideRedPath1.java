package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.vuforia;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmLevel;
import org.firstinspires.ftc.teamcode.commands.drive.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.webcam.CloseDetectTSEPosition;
import org.firstinspires.ftc.teamcode.commands.webcam.DetectTSEPosition;
import org.firstinspires.ftc.teamcode.commands.webcam.vuforia.CloseDetectVuforiaTSEPosition;
import org.firstinspires.ftc.teamcode.commands.webcam.vuforia.DetectVuforiaTSEPosition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.globals.Levels;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateArm;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateCarousel;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateIntake;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateVuforiaWebCam;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateWebCam;
import org.firstinspires.ftc.teamcode.subsystems.drive.roadrunner.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.VuforiaWebCamSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

public class DuckSideRedPath1 {

    private MecanumDriveSubsystem drive;
    private TrajectoryFollowerCommand sample1Follower1;
    private TrajectoryFollowerCommand sample1Follower2;
    private TrajectoryFollowerCommand sample1Follower3;
    private TrajectoryFollowerCommand sample1Follower4;

    private SequentialCommandGroup carouselGroupBlue1;
    private SequentialCommandGroup intakeGroup;

    private final Pose2d startPose;
    private final HardwareMap hwMap;
    private final Telemetry telemetry;

    private CreateIntake createIntake;
    private CloseDetectVuforiaTSEPosition closeDetectVuforiaTSEPosition;
    private InstantCommand stopDetect;

    public DuckSideRedPath1(HardwareMap hwMap, Pose2d sp, Telemetry telemetry){
        this.hwMap = hwMap;
        startPose = sp;
        this.telemetry = telemetry;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hwMap), false);

    }


    public void createPath(){
        //startPose = new Pose2d(-36, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        CreateCarousel createCarousel = new CreateCarousel(hwMap,"carousel",telemetry);
        CreateVuforiaWebCam createWebCam = new CreateVuforiaWebCam(hwMap, "Webcam 1", telemetry);
        CreateArm createArm = new CreateArm(hwMap, "arm", telemetry);

        createArm.createAuto();

        createWebCam.createAuto();
        VuforiaWebCamSubsystem webCamSubsystem = createWebCam.getVuforiaWebCamSubsystem();


        telemetry.addLine("Detecting Position");
        DetectVuforiaTSEPosition detectTSEPosition = createWebCam.getDetectTSEPositionCommand();
        closeDetectVuforiaTSEPosition = createWebCam.getCloseDetectTSEPosition();
        detectTSEPosition.schedule();

        createCarousel.createAuto();
        carouselGroupBlue1 = new SequentialCommandGroup(createCarousel.getMoveCarouselToPosition(),
                new WaitUntilCommand(createCarousel.hasMaxEncoderCountSupplier()).andThen(createCarousel.getStopCarousel()));

        createIntake = new CreateIntake(hwMap, "intake", telemetry);
        createIntake.createAuto();

        intakeGroup = new SequentialCommandGroup(
                createIntake.getSeGrabber(),
                new WaitCommand(800)
                        .andThen(createIntake.getStopIntake())
        );

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                //.strafeTo(new Vector2d(-60, 60))
                .splineToLinearHeading(new Pose2d(-63, -53.4, Math.toRadians(-65)),Math.toRadians(180))
                .addDisplacementMarker(()-> {
                    telemetry.addData("Path 1", "performing path 1 action");
                })
                .build();


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-60, -22))
                .addDisplacementMarker(()->{
                    telemetry.addData("Path 2 Set Level", Levels.getInstance().getTSELevel());
                    SetArmLevel setArmLevel = createArm.createSetArmLevel(Levels.getInstance().getTSELevel());
                    setArmLevel.schedule();
                })

                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())

                .splineToLinearHeading(new Pose2d(-35, -21.5, Math.toRadians(0)),Math.toRadians(90))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .addDisplacementMarker(()->{
                    SetArmLevel setArmLevel = createArm.createSetArmLevel(0);
                    setArmLevel.schedule();
                })
                .strafeTo(new Vector2d(-39,-22))
                .splineToLinearHeading(new Pose2d(-60, -33.5, Math.toRadians(0)),Math.toRadians(90))
                .build();

        sample1Follower1 = new TrajectoryFollowerCommand(drive,traj1);
        sample1Follower2 = new TrajectoryFollowerCommand(drive,traj2);
        sample1Follower3 = new TrajectoryFollowerCommand(drive,traj3);
        sample1Follower4 = new TrajectoryFollowerCommand(drive,traj4);

        stopDetect = new InstantCommand(()->{
            closeDetectVuforiaTSEPosition.schedule();
        });

    }

    public void execute(CommandOpMode commandOpMode){
        commandOpMode.schedule(new WaitUntilCommand(commandOpMode::isStarted).andThen(
                stopDetect.andThen(sample1Follower1,carouselGroupBlue1,sample1Follower2, sample1Follower3, intakeGroup, sample1Follower4)
        ));
    }
}

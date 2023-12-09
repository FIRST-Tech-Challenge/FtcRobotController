package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.teamProp.TeamPropDetection;
import org.firstinspires.ftc.teamcode.tools.Robot;
import org.firstinspires.ftc.teamcode.tools.TelemetryManager;

//@Autonomous(name="Autonomous Base")
public abstract class AutoBase extends LinearOpMode {

    protected StandardTrackingWheelLocalizer myLocalizer;

    public class Coordinates{
        Pose2d rightParkIntermediateBlueLeft = new Pose2d(-11.5, 42, Math.toRadians(270.00));
        Pose2d rightParkFinalBlueLeft = new Pose2d(-11.5, 50, Math.toRadians(270.00));
        Pose2d leftParkIntermediateRedRight = new Pose2d(11.5, 42, Math.toRadians(270.00));
        Pose2d leftParkFinalRedRight = new Pose2d(11.5, 50, Math.toRadians(270.00));

        //left backdrop
        Pose2d leftBackdropLeft = new Pose2d(-46, 50, Math.toRadians(270.00));
        Pose2d leftBackdropCenter = new Pose2d(-36, 50, Math.toRadians(270.00));
        Pose2d leftBackdropRight = new Pose2d(-28, 50, Math.toRadians(270.00));
        Pose2d leftBackdropIntermediateLeft = new Pose2d(-42, 30, Math.toRadians(270));
        Pose2d leftBackdropIntermediateCenter = new Pose2d(-36, 35, Math.toRadians(270));
        Pose2d leftBackdropIntermediateRight = new Pose2d(-30, 30, Math.toRadians(270));

        //right backdrop
        Pose2d rightBackdropLeft = new Pose2d(27, 50, Math.toRadians(270.00));
        Pose2d rightBackdropCenter = new Pose2d(36, 50, Math.toRadians(270.00));
        Pose2d rightBackdropRight = new Pose2d(46, 50, Math.toRadians(270.00));
        Pose2d rightBackdropIntermediateLeft = new Pose2d(30, 30, Math.toRadians(270));
        Pose2d rightBackdropIntermediateCenter = new Pose2d(36, 35, Math.toRadians(270));
        Pose2d rightBackdropIntermediateRight = new Pose2d(42, 30, Math.toRadians(270));


        //Blue Left
        Pose2d preStartPoseBlueLeft = new Pose2d(-63, 9.5, Math.toRadians(180)); //robot needs to strafe 2 inches to the actual start pose
        Pose2d startPoseBlueLeft = new Pose2d(-63, 14, Math.toRadians(180));
        Pose2d rightTeamPropBlueLeft = new Pose2d(-32, 10.5, Math.toRadians(90.00));
        Pose2d centerTeamPropBlueLeft = new Pose2d(-34.5, 12, Math.toRadians(180.00));
        Pose2d leftTeamPropBlueLeft = new Pose2d(-28, 9.5, Math.toRadians(270.00));

        //Blue right
        Pose2d preStartPoseBlueRight = new Pose2d(-63, -9.5, Math.toRadians(180));
        Pose2d startPoseBlueRight = new Pose2d(-61, -14, Math.toRadians(180));
        Pose2d leftTeamPropBlueRight = new Pose2d(-28, -13, Math.toRadians(270));
        Pose2d centerTeamPropBlueRight = new Pose2d(-35, -12, Math.toRadians(180.00));
        Pose2d rightTeamPropBlueRight = new Pose2d(-39, -11.5, Math.toRadians(90));

        //Red left
        Pose2d preStartPoseRedLeft = new Pose2d(63, -9.5, Math.toRadians(0));
        Pose2d startPoseRedLeft = new Pose2d(63, -13, Math.toRadians(0));
        Pose2d rightTeamPropRedLeft = new Pose2d(32, -11, Math.toRadians(270.00));
        Pose2d centerTeamPropRedLeft = new Pose2d(34.5, -12, Math.toRadians(0));
        Pose2d leftTeamPropRedLeft = new Pose2d(30, -10.5, Math.toRadians(90));

        //Red right
        Pose2d preStartPoseRedRight = new Pose2d(63, 9.5, Math.toRadians(0));
        Pose2d startPoseRedRight = new Pose2d(60, 15.5, Math.toRadians(0));
        Pose2d rightTeamPropRedRight = new Pose2d(28, 10.5, Math.toRadians(270));
        Pose2d centerTeamPropRedRight = new Pose2d(35, 12, Math.toRadians(0));
        Pose2d leftTeamPropRedRight = new Pose2d(31, 10, Math.toRadians(80));


    }
    Coordinates c = new Coordinates();
    static final double SLOWERVELOCITY = 15;
    static final double SLOWERANGULARVELOCITY = 2.5;

    public abstract void runAutonomous(Robot robot, SampleMecanumDrive drive, TeamPropDetection.propLocation propLoc);

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryManager.setTelemetry(telemetry);
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2, true);

        TeamPropDetection teamPropDetection = new TeamPropDetection();
        teamPropDetection.Setup(hardwareMap, telemetry);
        telemetry.setMsTransmissionInterval(50);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // hardware map for odometry encoders
        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, null, null);
        // start location (coordinate)


        // Let's have at list 33% chance to pick it right if nothing works
        TeamPropDetection.propLocation propLoc = TeamPropDetection.propLocation.CENTER;

        Robot.clawGrip.setPosition(Robot.clawClose);
        Robot.clawPitch.setPosition(Robot.clawPitchIntake);
        Robot.clawYaw.setPosition(Robot.clawYawIntake);


        while (!isStarted() && !isStopRequested())
        {
            TeamPropDetection.propLocation currentPropLoc = teamPropDetection.GetPropLocation();
            if(currentPropLoc!=TeamPropDetection.propLocation.NULL) {
                propLoc = currentPropLoc;
                telemetry.addLine("Detected:" + propLoc);
                telemetry.update();
            }
        }


        waitForStart();

        runAutonomous(robot, drive, propLoc);
    }

}

package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Autonomous
public class PurePursuitSample extends LinearOpMode {

    // define our constants

    private OTOSOdometry m_robotOdometry;
    private OdometrySubsystem m_odometry;
    private PurePursuitCommand ppCommand;
    private MecanumDrive m_robotDrive;
    private Motor fL, fR, bL, bR;
    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    public void run() {
        CommandScheduler.getInstance().run();
    }

    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    public void register(Subsystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        //m_robotDrive.driveWithMotorPowers(0.3, 0.3, 0.3, 0.3);
        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            FtcDashboard.getInstance().getTelemetry().addData("x", m_odometry.getPose().getX());
            FtcDashboard.getInstance().getTelemetry().addData("y", m_odometry.getPose().getY());
            FtcDashboard.getInstance().getTelemetry().addData("r", m_odometry.getPose().getRotation());
            FtcDashboard.getInstance().getTelemetry().update();

            run();
        }
        reset();
    }

    public static void disable() {
        Robot.disable();
    }

    public static void enable() {
        Robot.enable();
    }

    public void initialize() {
        fL = new Motor(hardwareMap, "leftFront");
        fR = new Motor(hardwareMap, "rightFront");
        bL = new Motor(hardwareMap, "leftBack");
        bR = new Motor(hardwareMap, "rightBack");

        // create our drive object
        m_robotDrive = new MecanumDrive(false, fL, fR, bL, bR);
        m_robotDrive.setRightSideInverted(false);

        // calculate multiplier
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
        drive.updatePoseEstimate();

        // create our odometry object and subsystem
        m_robotOdometry = new OTOSOdometry(drive);
        m_odometry = new OdometrySubsystem(m_robotOdometry);


        // create our pure pursuit command
        ppCommand = new PurePursuitCommand(
                m_robotDrive, m_odometry,
                new StartWaypoint(0, 0),
                new GeneralWaypoint(24, 0, 0.8, 0.8, 8),
                new EndWaypoint(
                        -24, 0, 0, 0.5,
                        0.5, 3, 0.8, 1
                )
        );
        // schedule the command
        schedule(ppCommand);
    }

}
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.freightfrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.freightfrenzy.RoadRunnerSubsystem;
import org.firstinspires.ftc.teamcode.myroadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opencvpipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.powerplay.AprilTagDetectionSubsystem;
import org.firstinspires.ftc.teamcode.robotbase.RobotEx;

@Autonomous(name = "Season: Freight Frenzy Autonomous Plan A _ NoOtherSide")
public class FreightFrenzyAutonomousPlanA_NoOtherSide extends CommandOpMode {

    FreightFrenzyRobot robot;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    protected ElapsedTime runtime;
    protected SampleMecanumDrive drive;
    protected RoadRunnerSubsystem RR;
    protected AprilTagDetectionSubsystem april_tag;

    @Override
    public void initialize() {
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        robot = new FreightFrenzyRobot(hardwareMap, telemetry, driverOp, toolOp,
                RobotEx.OpModeType.AUTO, true, false,
                false, false, false, false, true);

        drive = new SampleMecanumDrive(hardwareMap);

        RR = new RoadRunnerSubsystem(drive, hardwareMap, false);

        april_tag = new AprilTagDetectionSubsystem(robot.camera);

        runtime = new ElapsedTime();
    }

    public void waitForStart() {
        /////////////////////////////////// Recognizing the Tag ///////////////////////////////////
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            april_tag.aprilTagCheck();
            sleep(20);
        }
    }

    @Override
    public void run() {
        super.run();
        // TODO: Make telemetry subsystem/command and remove this function
        robot.telemetryUpdate();
        robot.dashboardTelemetryUpdate();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        ///////////////////////////////// Running the Trajectories /////////////////////////////////

        int i = 0;

        if (isStopRequested()) return;

        RR.runHS();

        while (runtime.seconds() < 25){

//            RR.runSCORING();
            sleep(5000);

        }

        if (april_tag.getTagOfInterest().id == april_tag.LEFT) RR.runP1();
        else if (april_tag.getTagOfInterest().id == april_tag.RIGHT|| april_tag.getTagOfInterest() == null) RR.runP3();
        else RR.runTOMID() ;

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
    }
}
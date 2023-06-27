package org.firstinspires.ftc.teamcode;

import static org.inventors.robotbase.RobotEx.OpModeType.AUTO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.robotbase.MecanumDrivePPV2;
import org.firstinspires.ftc.teamcode.powerplayV2.AprilTagDetectionSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.PowerPlayRobotV2;
import org.firstinspires.ftc.teamcode.powerplayV2.RoadRunnerSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.BasketSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ConeDetectorSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.FrontSliderSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.LimitSwitchSubsystem;
import org.inventors.robotbase.GamepadExEx;

import java.util.HashMap;

@Autonomous(name = "TestAuto", group = "TestDeAutonomi")
public class AutonomousTests extends CommandOpMode {
    PowerPlayRobotV2 robot;
    protected MecanumDrivePPV2 drive;
    protected RoadRunnerSubsystem RR;
    protected AprilTagDetectionSubsystem april_tag;
    protected ElevatorSubsystem elevator;
    protected BasketSubsystem basket;
    protected ArmSubsystem arm;
    protected boolean april_tag_found = false;
    Telemetry dashboardTelemetry;
    protected SequentialCommandGroup scoringCommand;
    @Override
    public void initialize() {
        GamepadExEx driverOp = new GamepadExEx(gamepad1);
        GamepadExEx toolOp = new GamepadExEx(gamepad2);

        robot = new PowerPlayRobotV2(hardwareMap, telemetry, driverOp, toolOp, AUTO, true,
                false);

        drive = new MecanumDrivePPV2(hardwareMap, AUTO);

        RR = new RoadRunnerSubsystem(drive, false);

        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        april_tag = new AprilTagDetectionSubsystem(robot.camera, dashboardTelemetry);

    }

    public void waitForStart() {
        /////////////////////////////////// Recognizing the Tag ///////////////////////////////////
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
//        while (!isStarted() && !isStopRequested()) {
//            april_tag_found = april_tag.aprilTagCheck();
////            if (april_tag.getTagOfInterest() == null)
////                april_tag_not_found = true;
//            sleep(20);
//        }
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
        if (isStopRequested()) return;

        RR.runTEST();


        //Select Command Auto
//        new Trigger(() -> runtime.seconds() >= 20).whenActive(
//                new SelectCommand(
//                        new HashMap<Object, Command>() {{
//                            put(april_tag.LEFT, new InstantCommand(RR::runP1, RR));
//                            put(april_tag.MIDDLE, new InstantCommand(RR::runTOMID, RR));
//                            put(april_tag.RIGHT, new InstantCommand(RR::runP3, RR));
//                            put(-1, new InstantCommand(RR::runTOMID, RR));
//                        }},
//                        () -> april_tag_found ? april_tag.getTagOfInterest().id : -1
//                )
//        );

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
            run();
        }
        reset();
    }
}
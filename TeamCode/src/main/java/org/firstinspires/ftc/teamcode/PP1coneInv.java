package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.robotbase.RobotEx.OpModeType.AUTO;

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

import org.firstinspires.ftc.teamcode.myroadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opencvpipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.powerplayV2.AprilTagDetectionSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.PowerPlayRobotV2;
import org.firstinspires.ftc.teamcode.powerplayV2.RoadRunnerSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.BasketSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.robotbase.GamepadExEx;

import java.util.HashMap;

@Autonomous(name = "PP1coneInv", group = "Final Autonomous")
public class PP1coneInv extends CommandOpMode {

    PowerPlayRobotV2 robot;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    protected ElapsedTime runtime;
    protected SampleMecanumDrive drive;
    protected RoadRunnerSubsystem RR;
    protected AprilTagDetectionSubsystem april_tag;
    protected ClawSubsystem claw;
    protected ElevatorSubsystem elevator;
    protected BasketSubsystem basket;
    protected ArmSubsystem arm;

    protected SequentialCommandGroup scoringCommand;

    @Override
    public void initialize() {
        GamepadExEx driverOp = new GamepadExEx(gamepad1);
        GamepadExEx toolOp = new GamepadExEx(gamepad2);

        robot = new PowerPlayRobotV2(hardwareMap, telemetry, driverOp, toolOp, AUTO, true,
                false, false, false, false, false);

        drive = new SampleMecanumDrive(hardwareMap);

        RR = new RoadRunnerSubsystem(drive, hardwareMap, true);

        april_tag = new AprilTagDetectionSubsystem(robot.camera, telemetry);

        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        basket =  new BasketSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap, telemetry);

        scoringCommand = new SequentialCommandGroup(
                new InstantCommand(arm::setMid, arm),
                new WaitCommand(300),
                new ElevatorCommand(elevator, ElevatorSubsystem.Level.HIGH),
                new InstantCommand(basket::setOuttake, basket),
                new WaitCommand(1500),
                new ParallelCommandGroup(
                        new InstantCommand(basket::setTravel, basket),
                        new ElevatorCommand(elevator, ElevatorSubsystem.Level.LOW)
                ));


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
        runtime.reset();

        ///////////////////////////////// Running the Trajectories /////////////////////////////////

        int i = 0;

        if (isStopRequested()) return;

        schedule(new SequentialCommandGroup(
                new InstantCommand(RR::runHS, RR),
                scoringCommand
        ));

        //Select Command Auto
        new Trigger(() -> runtime.seconds() >= 20).whenActive(
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(april_tag.LEFT, new InstantCommand(RR::runP1, RR));
                            put(april_tag.MIDDLE, new InstantCommand(RR::runTOMID, RR));
                            put(april_tag.RIGHT, new InstantCommand(RR::runP3, RR));
                        }},
                        () -> april_tag.getTagOfInterest().id
                )
        );

//        if (runtime.seconds() >= 20) {
//            if (april_tag.getTagOfInterest().id == april_tag.LEFT) RR.runP1();
//            else if (april_tag.getTagOfInterest().id == april_tag.RIGHT || april_tag.getTagOfInterest() == null)
//                RR.runP3();
//            else RR.runTOMID();
//        }

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.update();
            run();
        }
        reset();
    }
}
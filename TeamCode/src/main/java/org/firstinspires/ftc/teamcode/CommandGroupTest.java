package org.firstinspires.ftc.teamcode;

import static org.inventors.ftc.robotbase.RobotEx.OpModeType.AUTO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.MecanumDrivePPV2;
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
import org.inventors.ftc.robotbase.GamepadExEx;

@Autonomous(name = "TestAutonomous", group = "Final Autonomous")
public class CommandGroupTest extends CommandOpMode {
    PowerPlayRobotV2 robot;
    protected ElapsedTime runtime;
    protected MecanumDrivePPV2 drive;
    protected RoadRunnerSubsystem RR;
    protected AprilTagDetectionSubsystem april_tag;
    protected ClawSubsystem claw;
    protected ElevatorSubsystem elevator;
    protected BasketSubsystem basket;
    protected ArmSubsystem arm;
    protected FrontSliderSubsystem frontSlider;
    protected LimitSwitchSubsystem rightSwitch, leftSwitch;
    protected ConeDetectorSubsystem cone_detector;
    protected boolean april_tag_found = false;
    Telemetry dashboardTelemetry;
    protected SequentialCommandGroup scoringCommand, test;
    @Override
    public void initialize() {
        GamepadExEx driverOp = new GamepadExEx(gamepad1);
        GamepadExEx toolOp = new GamepadExEx(gamepad2);

        robot = new PowerPlayRobotV2(hardwareMap, telemetry, driverOp, toolOp, AUTO, true,
                false);

        drive = new MecanumDrivePPV2(hardwareMap, AUTO);

        RR = new RoadRunnerSubsystem(drive, false);

        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        basket =  new BasketSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap, telemetry);
        rightSwitch = new LimitSwitchSubsystem(hardwareMap, "rightSwitch");
        leftSwitch = new LimitSwitchSubsystem(hardwareMap, "leftSwitch");
        frontSlider = new FrontSliderSubsystem(hardwareMap, () -> rightSwitch.getState(),
                () -> leftSwitch.getState(), telemetry);

        scoringCommand = new SequentialCommandGroup(
                new InstantCommand(arm::setMid, arm),
                new WaitCommand(300),
                new ElevatorCommand(elevator, ElevatorSubsystem.Level.AUTO_SCORING),
                new InstantCommand(basket::setOuttake, basket),
                new WaitCommand(1500),
                new ParallelCommandGroup(
                        new InstantCommand(basket::setTravel, basket),
                        new InstantCommand(arm::setMid, arm)
                ),
                new ElevatorCommand(elevator, ElevatorSubsystem.Level.LOW)
        );
        runtime = new ElapsedTime();
    }

    public void waitForStart() {
        /////////////////////////////////// Recognizing the Tag ///////////////////////////////////
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
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
        if (isStopRequested()) return;

        test = new SequentialCommandGroup(
                new InstantCommand(arm::setIntake, arm),
                new WaitCommand(1500),
                new InstantCommand(arm::setTravel, arm),
                scoringCommand,
                new WaitCommand(1500),
                new InstantCommand(arm::setIntake, arm)
        );

        schedule(test);

/*        schedule(new SequentialCommandGroup(
                new InstantCommand(arm::setIntake, arm),
                new WaitCommand(1500),
                new InstantCommand(arm::setTravel, arm),
                scoringCommand,
                new WaitCommand(1500),
                new InstantCommand(arm::setIntake, arm)
        ));*/
/*
        schedule(new SequentialCommandGroup(
                        new InstantCommand(RR::runHS2, RR),
                        new InstantCommand(arm::setMid, arm),
                        new WaitCommand(300),
                        new ElevatorCommand(elevator, ElevatorSubsystem.Level.AUTO_SCORING),
                        new InstantCommand(basket::setOuttake, basket),
                        new WaitCommand(1500),
                        new ParallelCommandGroup(
                                new InstantCommand(basket::setTravel, basket),
                                new ElevatorCommand(elevator, ElevatorSubsystem.Level.LOW),
                                new InstantCommand(claw::release, claw),
                                new InstantCommand(() -> arm.setAutonomousPosition(0), arm)
                        ),
                        new FrontSliderConeCommand(frontSlider, cone_detector::isConeDetected, arm),
                        new InstantCommand(claw::grab, claw),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new InstantCommand(arm::setTravel, arm),
                                new InstantCommand(frontSlider::close, frontSlider)
                        ),
                        new WaitCommand(800),
                        new InstantCommand(claw::release, claw), // Release the cone to tha basket
                        new WaitCommand(500),
                        new ElevatorCommand(elevator, ElevatorSubsystem.Level.TRAVEL),
                        new WaitCommand(100),
                        new InstantCommand(() -> frontSlider.manual(0.4), frontSlider),
                        new WaitCommand(400),
                        new InstantCommand(() -> frontSlider.stop(), frontSlider),
                        new ParallelCommandGroup(
                                new InstantCommand(arm::setMid, arm),
                                new InstantCommand(frontSlider::close, frontSlider)
                        ),
                        new WaitCommand(600),
                        new ElevatorCommand(elevator, ElevatorSubsystem.Level.AUTO_SCORING),
                        new InstantCommand(basket::setOuttake, basket),
                        new WaitCommand(1500),
                        new ParallelCommandGroup(
                                new InstantCommand(basket::setTravel, basket),
                                new ElevatorCommand(elevator, ElevatorSubsystem.Level.LOW)
                        )
                )
        );

 */

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.update();
            run();
        }
        reset();
    }
}
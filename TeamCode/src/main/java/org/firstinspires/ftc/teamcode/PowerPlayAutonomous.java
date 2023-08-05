package org.firstinspires.ftc.teamcode;

import static org.inventors.ftc.robotbase.RobotEx.OpModeType.AUTO;

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
import org.inventors.ftc.robotbase.MecanumDrivePPV2;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.AprilTagDetectionSubsystem;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.SlidyRobot;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.RoadRunnerSubsystem;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.BasketSubsystem;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.ConeDetectorSubsystem;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.FrontSliderSubsystem;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.LimitSwitchSubsystem;
import org.inventors.ftc.robotbase.GamepadExEx;

import java.util.HashMap;

@Autonomous(name = "AutoOneCone", group = "Final Autonomous")
public class PowerPlayAutonomous extends CommandOpMode {
    SlidyRobot slidy;
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
    protected SequentialCommandGroup scoringCommand;
    @Override
    public void initialize() {
        GamepadExEx driverOp = new GamepadExEx(gamepad1);
        GamepadExEx toolOp = new GamepadExEx(gamepad2);

        slidy = new SlidyRobot(hardwareMap, telemetry, driverOp, toolOp, AUTO, true,
                false);

        drive = new MecanumDrivePPV2(hardwareMap, AUTO);

        RR = new RoadRunnerSubsystem(drive, false);

        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        april_tag = new AprilTagDetectionSubsystem(slidy.camera, dashboardTelemetry);

        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        basket =  new BasketSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap, telemetry);
        rightSwitch = new LimitSwitchSubsystem(hardwareMap, "rightSwitch");
        leftSwitch = new LimitSwitchSubsystem(hardwareMap, "leftSwitch");
        frontSlider = new FrontSliderSubsystem(hardwareMap, () -> rightSwitch.getState(),
                () -> leftSwitch.getState(), telemetry);
        cone_detector = new ConeDetectorSubsystem(hardwareMap, 30);

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
            april_tag_found = april_tag.aprilTagCheck();
//            if (april_tag.getTagOfInterest() == null)
//                april_tag_not_found = true;
            sleep(20);
        }
    }

    @Override
    public void run() {
        super.run();
        // TODO: Make telemetry subsystem/command and remove this function
        slidy.telemetryUpdate();
        slidy.dashboardTelemetryUpdate();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        runtime.reset();

        ///////////////////////////////// Running the Trajectories /////////////////////////////////
        if (isStopRequested()) return;

        schedule(new SequentialCommandGroup(
                new InstantCommand(RR::runHS2, RR),
                scoringCommand
        ));
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

        //Select Command Auto
        new Trigger(() -> runtime.seconds() >= 20).whenActive(
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(april_tag.LEFT, new InstantCommand(RR::runP1, RR));
                            put(april_tag.MIDDLE, new InstantCommand(RR::runTOMID, RR));
                            put(april_tag.RIGHT, new InstantCommand(RR::runP3, RR));
                            put(-1, new InstantCommand(RR::runTOMID, RR));
                        }},
                        () -> april_tag_found ? april_tag.getTagOfInterest().id : -1
                )
        );

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.update();
            run();
        }
        reset();
    }
}
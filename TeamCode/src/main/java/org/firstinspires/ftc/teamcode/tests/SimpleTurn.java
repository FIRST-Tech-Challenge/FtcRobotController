package org.firstinspires.ftc.teamcode.tests;

import static org.inventors.ftc.robotbase.RobotEx.OpModeType.AUTO;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Slidy_PPV2.SlidyRobot;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.RoadRunnerSubsystem;
import org.inventors.ftc.robotbase.GamepadExEx;
import org.inventors.ftc.robotbase.MecanumDrivePPV2;

@Autonomous(name = "SimpleTurn", group = "Tests")
public class SimpleTurn extends CommandOpMode {
    SlidyRobot slidy;
    protected ElapsedTime runtime;
    protected MecanumDrivePPV2 drive;
    protected RoadRunnerSubsystem RR;
    @Override
    public void initialize() {
        GamepadExEx driverOp = new GamepadExEx(gamepad1);
        GamepadExEx toolOp = new GamepadExEx(gamepad2);

        slidy = new SlidyRobot(hardwareMap, telemetry, driverOp, toolOp, AUTO, true,
                false);

        drive = new MecanumDrivePPV2(hardwareMap, AUTO);

        RR = new RoadRunnerSubsystem(drive, false);

        runtime = new ElapsedTime();
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

        if (isStopRequested()) return;

        schedule(new InstantCommand(RR::runTEST, RR));

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Runtime", runtime.seconds());
            run();
        }
        reset();
    }
}
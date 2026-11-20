package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Tests stage-1 loading and timed gate-open feeding (both linked transfer motors), without a flywheel. */
@TeleOp(name = "Test: Transfer Only", group = "Test")
public class TransferTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        GateSubsystem gate = new GateSubsystem(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);
        telemetry.addLine("transfer_motor_1: Expansion Hub motor/encoder port 2, FORWARD, full power");
        telemetry.addLine("transfer_motor_2: Expansion Hub motor port 3, FORWARD, no encoder | linked to stage 1, same power");
        telemetry.addLine("distance_sensor: Expansion Hub digital port 0, Digital Device input");
        telemetry.addLine("After START: stage 1 (both motors) loads | 0.5 s HIGH = full, then 0.5 s stage-1 run-on | LOW resumes");
        telemetry.addLine("Hold RB = Timed Gate: stage 1 feeds, and runs while gate closed between shots | LB = Pause / Close Gate (priority)");
        telemetry.addLine("Transfer-only feed test: flywheel is NOT initialized or powered; start empty.");
        transfer.addTelemetry(telemetry);
        loop.addTelemetry(telemetry); telemetry.update();

        waitForStart();
        FiringCoordinator firing = ShootingConfiguration.firing();
        StaggeredFeedTiming stage1Timing = new StaggeredFeedTiming();

        try {
            while (opModeIsActive()) {
                loop.begin();
                double now = System.nanoTime() * 1e-9;
                // RB deliberately exercises the shared timed sequence without shooter readiness.
                firing.update(now, gamepad1.right_bumper, false, gamepad1.left_bumper,
                        false, true, true, true, "");
                gate.setOpen(firing.gateOpen());
                boolean stage1 = stage1Timing.feedStage1(now, firing);
                boolean closedGateStage1 = firing.closedGateStage1() && !gamepad1.left_bumper;
                transfer.update(now, firing.feeding() && gate.isOpenForFeeding(),
                        !closedGateStage1 && (gamepad1.left_bumper || gamepad1.right_bumper),
                        stage1, closedGateStage1);
                transfer.addStatusTelemetry(telemetry);
                telemetry.addData("Transfer feed test", "%s | %s", firing.state(), firing.reason());
                telemetry.addData("Timed shooting (s)", "gate %.2f | feed %.2f | recovery %.2f",
                        firing.gateTravelSeconds, firing.feedWindowSeconds, firing.recoverySeconds);
                gate.addTelemetry(telemetry);
                transfer.addTelemetry(telemetry);
                loop.addTelemetry(telemetry); telemetry.update();
            }
        } finally {
            RobotShutdown.stopAll(transfer::stop, gate::close);
        }
    }
}

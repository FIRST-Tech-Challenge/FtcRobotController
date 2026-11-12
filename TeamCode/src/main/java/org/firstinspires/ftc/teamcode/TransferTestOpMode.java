package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Tests stage-1-only loading and timed gate-open feeding of both stages, without a flywheel. */
@TeleOp(name = "Test: Transfer Only", group = "Test")
public class TransferTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        GateSubsystem gate = new GateSubsystem(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);
        telemetry.addLine("transfer_motor_1: Expansion Hub motor port 2, REVERSE, no encoder");
        telemetry.addLine("transfer_motor_2: Expansion Hub motor port 3, FORWARD, no encoder | off during loading");
        telemetry.addLine("distance_sensor: Expansion Hub digital port 0, Digital Device input");
        telemetry.addLine("After START: stage 1 loads, stage 2 off | 0.2 s HIGH = full, then 0.5 s stage-1 run-on | LOW resumes");
        telemetry.addLine("Hold RB = Timed Gate + Both Stages | LB = Pause / Close Gate (priority)");
        telemetry.addLine("Transfer-only feed test: flywheel is NOT initialized or powered; start empty.");
        transfer.addTelemetry(telemetry);
        loop.addTelemetry(telemetry); telemetry.update();

        waitForStart();
        FiringCoordinator firing = ShootingConfiguration.firing();

        try {
            while (opModeIsActive()) {
                loop.begin();
                double now = System.nanoTime() * 1e-9;
                // RB deliberately exercises the shared timed sequence without shooter readiness.
                firing.update(now, gamepad1.right_bumper, false, gamepad1.left_bumper,
                        false, true, true, true, "");
                gate.setOpen(firing.gateOpen());
                transfer.update(firing.feeding() && gate.isOpenForFeeding(),
                        gamepad1.left_bumper || gamepad1.right_bumper);
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

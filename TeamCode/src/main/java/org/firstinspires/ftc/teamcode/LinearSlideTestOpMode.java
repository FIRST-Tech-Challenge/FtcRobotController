package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Tests only the linear slide and its claw, with the competition toggles plus measuring tools.
 * Start with the slide fully down.
 *
 * <ul>
 *     <li><b>GP2 Left Stick Click</b>: toggle slide fully down / fully up (same as competition)</li>
 *     <li><b>GP1 B</b>: toggle claw open / closed (same as competition)</li>
 *     <li><b>GP2 Left Stick Y (hold)</b>: manual slide power at half scale, no limits; push up = up.
 *         Release leaves it unpowered until the next toggle. Use it to find the top.</li>
 *     <li><b>GP2 Back</b>: re-zero: current position becomes fully down</li>
 *     <li><b>GP1 D-Pad Up / Down</b>: nudge claw position +/-0.01 to find open and closed values</li>
 * </ul>
 */
@TeleOp(name = "Test: Linear Slide + Claw", group = "Test")
public class LinearSlideTestOpMode extends LinearOpMode {

    public static final double MANUAL_POWER_SCALE = 0.5;
    public static final double STICK_DEADBAND = 0.1;
    public static final double CLAW_NUDGE_STEP = 0.01;

    @Override
    public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        LinearSlideSubsystem slide = null;
        try {
            slide = new LinearSlideSubsystem(hardwareMap);
            SlideClawSubsystem claw = new SlideClawSubsystem(hardwareMap);

            while (opModeInInit()) {
                loop.begin();
                slide.trackWithoutDriving();
                telemetry.addLine("Slide must be FULLY DOWN now (INIT = zero).");
                telemetry.addLine("Exp Hub servo 5: slide_servo (CR) | Exp Hub analog 0: slide_encoder | CH servo 0: claw_servo");
                slide.addTelemetry(telemetry);
                claw.addTelemetry(telemetry);
                loop.addTelemetry(telemetry); telemetry.update();
            }
            if (isStopRequested()) return;

            boolean previousSlideToggle = false;
            boolean previousClawToggle = false;
            boolean previousRezero = false;
            boolean previousNudgeUp = false;
            boolean previousNudgeDown = false;
            while (opModeIsActive()) {
                loop.begin();

                boolean slideToggle = gamepad2.left_stick_button;
                if (slideToggle && !previousSlideToggle) slide.toggle();
                previousSlideToggle = slideToggle;

                double manual = -gamepad2.left_stick_y;
                slide.setManualPower(Math.abs(manual) > STICK_DEADBAND ? manual * MANUAL_POWER_SCALE : Double.NaN);

                if (gamepad2.back && !previousRezero) slide.rezero();
                previousRezero = gamepad2.back;
                slide.update();

                boolean clawToggle = gamepad1.b;
                if (clawToggle && !previousClawToggle) claw.toggle();
                previousClawToggle = clawToggle;
                if (gamepad1.dpad_up && !previousNudgeUp)
                    claw.setPositionForTest(claw.getCommandedPosition() + CLAW_NUDGE_STEP);
                if (gamepad1.dpad_down && !previousNudgeDown)
                    claw.setPositionForTest(claw.getCommandedPosition() - CLAW_NUDGE_STEP);
                previousNudgeUp = gamepad1.dpad_up;
                previousNudgeDown = gamepad1.dpad_down;

                telemetry.addLine("GP2: LS Click = slide toggle | LS Y = manual | Back = re-zero");
                telemetry.addLine("GP1: B = claw toggle | D-Pad Up/Down = nudge claw");
                slide.addDriverTelemetry(telemetry);
                claw.addDriverTelemetry(telemetry);
                slide.addTelemetry(telemetry);
                claw.addTelemetry(telemetry);
                loop.addTelemetry(telemetry); telemetry.update();
            }
        } finally {
            RobotShutdown.stopAll(slide == null ? null : slide::stop);
        }
    }
}

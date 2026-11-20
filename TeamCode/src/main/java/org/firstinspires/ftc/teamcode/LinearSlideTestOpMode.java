package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Tests only the linear slide (shot blocker permanently attached, no claw), with the competition
 * controls plus measuring tools. Start with the slide fully down.
 *
 * <ul>
 *     <li><b>GP2 Right Stick Click</b>: extend slide fully up (same as competition)</li>
 *     <li><b>GP2 Left Stick Click</b>: home slide fully down (same as competition)</li>
 *     <li><b>GP2 Left Stick Y (hold)</b>: manual slide power at half scale, no limits; push up = up.
 *         Release leaves it unpowered until the next extend/home. Use it to find the top.</li>
 *     <li><b>GP2 Back</b>: re-zero: current position becomes fully down</li>
 * </ul>
 */
@TeleOp(name = "Test: Linear Slide", group = "Test")
public class LinearSlideTestOpMode extends LinearOpMode {

    public static final double MANUAL_POWER_SCALE = 0.5;
    public static final double STICK_DEADBAND = 0.1;

    @Override
    public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        LinearSlideSubsystem slide = null;
        try {
            slide = new LinearSlideSubsystem(hardwareMap);

            while (opModeInInit()) {
                loop.begin();
                slide.trackWithoutDriving();
                telemetry.addLine("Slide must be FULLY DOWN now (INIT = zero).");
                telemetry.addLine("Exp Hub servo 5: slide_servo (CR) | CH analog 1: slide_encoder");
                slide.addTelemetry(telemetry);
                loop.addTelemetry(telemetry); telemetry.update();
            }
            if (isStopRequested()) return;

            boolean previousSlideExtend = false;
            boolean previousSlideHome = false;
            boolean previousRezero = false;
            while (opModeIsActive()) {
                loop.begin();

                boolean slideExtend = gamepad2.right_stick_button;
                boolean slideHome = gamepad2.left_stick_button;
                if (slideHome && !previousSlideHome) slide.lower();
                else if (slideExtend && !previousSlideExtend && !slideHome) slide.raise();
                previousSlideExtend = slideExtend;
                previousSlideHome = slideHome;

                double manual = -gamepad2.left_stick_y;
                slide.setManualPower(Math.abs(manual) > STICK_DEADBAND ? manual * MANUAL_POWER_SCALE : Double.NaN);

                if (gamepad2.back && !previousRezero) slide.rezero();
                previousRezero = gamepad2.back;
                slide.update();

                telemetry.addLine("GP2: RS Click = extend | LS Click = home | LS Y = manual | Back = re-zero");
                slide.addDriverTelemetry(telemetry);
                slide.addTelemetry(telemetry);
                loop.addTelemetry(telemetry); telemetry.update();
            }
        } finally {
            RobotShutdown.stopAll(slide == null ? null : slide::stop);
        }
    }
}

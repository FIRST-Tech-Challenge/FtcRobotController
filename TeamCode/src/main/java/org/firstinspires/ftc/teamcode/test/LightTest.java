package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;
import org.firstinspires.ftc.teamcode.robot.IndicatorLight;

/**
 * Step through the RGB indicator light positions to find the value for each color.
 * Both lights (aimer_led and launcher_led) always show the same value.
 */
@TeleOp(name = "Light Test", group = "Test")
public class LightTest extends GreenBotBaseOpMode {

    private static final double BIG_STEP = 0.1;
    private static final double SMALL_STEP = 0.01;

    private double value = IndicatorLight.GREEN; // start on a color we know

    @Override
    public void runOpMode() {
        config();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) value += BIG_STEP;
            if (gamepad1.dpadDownWasPressed()) value -= BIG_STEP;
            if (gamepad1.dpadRightWasPressed()) value += SMALL_STEP;
            if (gamepad1.dpadLeftWasPressed()) value -= SMALL_STEP;

            // Keep it between 0 and 1, and round to 2 decimals so repeated steps
            // don't drift (0.1 + 0.2 = 0.30000000000000004 in floating point).
            value = Range.clip(Math.round(value * 100) / 100.0, 0, 1);

            aimerLed.setColor(value);
            launcherLed.setColor(value);

            telemetry.addData("Light value", "%.2f", value);
            telemetry.addLine();
            telemetry.addData("D-pad Up / Down", "+/- %.1f", BIG_STEP);
            telemetry.addData("D-pad Right / Left", "+/- %.2f", SMALL_STEP);
            telemetry.addLine();
            telemetry.addData("Current colors in IndicatorLight",
                    "OFF %.2f, RED %.2f, YELLOW %.2f, GREEN %.2f, BLUE %.2f, PINK %.2f",
                    IndicatorLight.OFF, IndicatorLight.RED, IndicatorLight.YELLOW,
                    IndicatorLight.GREEN, IndicatorLight.BLUE, IndicatorLight.PINK);
            telemetry.update();
        }
    }
}

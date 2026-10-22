package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.drivers.PrismLedDriver;

/**
 * The LED strip, on its own: the first six LEDs red, orange, yellow, green, blue, purple,
 * the rest dark. Nothing moves. This OpMode talks straight to the Prism driver, so the
 * whole program is in this one file: change the colors in the table below.
 *
 * <p>How it works: the Prism has ten layers. Each layer draws one animation on a run of
 * LEDs, from a start LED to a stop LED. Here each LED gets its own layer with a Solid
 * Color animation one LED long. LEDs that no layer paints stay dark.
 *
 * <p>The Driver Station shows what the Prism answered (its device id and status bytes),
 * which is the first thing to check if the strip stays dark.
 */
@TeleOp(name = "Test: Light Strip", group = "Test")
public class StripTest extends LinearOpMode {

    /** One row per LED, from the Prism outward: red, green, blue, each 0 to 255. */
    private static final int[][] COLORS = {
            {255, 0, 0},      // LED 0: red
            {255, 96, 0},     // LED 1: orange
            {255, 255, 0},    // LED 2: yellow
            {0, 255, 0},      // LED 3: green
            {0, 0, 255},      // LED 4: blue
            {128, 0, 255},    // LED 5: purple
    };

    @Override
    public void runOpMode() {
        BotProperties properties = new BotProperties();
        PrismLedDriver strip = hardwareMap.get(PrismLedDriver.class, properties.cfgStrip);
        int deviceId = strip.ReadDeviceId();
        byte[] status = strip.ReadStatus();

        // One layer per LED. Layer n paints LED n only: start at n, stop at n + 1.
        // CHECK on the strip: if each color covers two LEDs, change "led + 1" to "led".
//        for (int led = 0; led < COLORS.length; led++) {
//            int[] color = COLORS[led];
//            strip.SolidColor(led, led, led + 1, properties.stripBrightness, color[0], color[1], color[2]);
//        }
//        strip.SolidColor(0,0,1, properties.stripBrightness, 255, 0, 180);
//        strip.SolidColor(1,1,2, properties.stripBrightness, 0, 255, 209);
//        strip.SolidColor(2,2,3, properties.stripBrightness, 255, 149, 0);
        // The Prism has only 10 layers, so same-colored neighbors share one layer.
        int b = properties.stripBrightness;
        strip.SolidColor(0, 0, 3, b, 254, 255, 0);     // LEDs 0-1 yellow
        strip.SolidColor(1, 3, 4, b, 255, 0, 0);       // LEDs 2-3 red
//        strip.SolidColor(2, 4, 6, b, 0, 60, 255);      // LEDs 4-5 blue
//        strip.SolidColor(3, 6, 12, b, 255, 255, 255);  // LEDs 6-11 white
//        strip.SolidColor(4, 12, 14, b, 254, 255, 0);   // LEDs 12-13 yellow
//        strip.SolidColor(5, 14, 16, b, 255, 0, 0);     // LEDs 14-15 red
//        strip.SolidColor(6, 16, 18, b, 0, 60, 255);    // LEDs 16-17 blue
        // The layers after the colors stay empty.
        for (int layer = 1; layer < PrismLedDriver.LAYERS; layer++) {
            strip.ClearLayer(layer);
        }

        while (opModeInInit() || opModeIsActive()) {
            telemetry.addData("Strip device id", "0x%02X (register 0x00)", deviceId);
            telemetry.addData("Strip status bytes", "%02X %02X %02X %02X (LEDs %d if lowest byte first)",
                    status[0] & 0xFF, status[1] & 0xFF, status[2] & 0xFF, status[3] & 0xFF, status[0] & 0xFF);
            telemetry.addData("Lit", "%d LEDs: red, orange, yellow, green, blue, purple", COLORS.length);
            telemetry.addLine("The colors are the table at the top of StripTest.java.");
            telemetry.update();
            idle();
        }
    }
}

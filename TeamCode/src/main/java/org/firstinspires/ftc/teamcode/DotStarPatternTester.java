package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

/**
 *
 * ! Warning: Make sure you have updated your REV Robotics module to firmware 1.7.2 or greater.
 *
 * ! Warning: It's your job to ensure the LEDs don't draw too much current from your robot.
 *
 * @author Rick Van Smith
 * @version 1.0.0
 */
//@TeleOp(name = "DotStar Pattern Test", group = "Test")
public class DotStarPatternTester extends OpMode {
    DotStarBridgedLED leds;

    /* The patterns to run on our LEDs. */
    IDotStarPattern rainbow;
    IDotStarPattern rainbowShift;
    IDotStarPattern halfAndHalf;
    IDotStarPattern chase;
    IDotStarPattern twinkle;
    IDotStarPattern indicator;
    IDotStarPattern ledDisplay;
    IDotStarPattern workshop;
    float[] redHsv = new float[]{0f, 0f, 0f};
    float[] yellowHsv = new float[]{0f, 0f, 0f};
    float[] greenHsv = new float[]{0f, 0f, 0f};

    @Override
    public void init() {
        // Set up the LEDs. Change this to your configured name.
        leds = hardwareMap.get(DotStarBridgedLED.class, "leds");

        // Use ModernRoboticsDIM if using Modern Robotics hardware.
        leds.setController(DotStarBridgedLED.Controller.RevExpansionHub);

        // Set the length of the strip.
        leds.setLength(60);

        // Create the twinkle pattern to run
        twinkle = new DSPatternTwinkle(leds);

        // Create the rainbow patterns to run.
        rainbow = new DSPatternRainbow(leds);
        rainbowShift = new DSPatternRainbowShift(leds);

        // We used Half and Half to indicate our collector contents, gold for
        // gold and silver for silver.  Since we had two collectors we split
        // the lights in half.
        halfAndHalf = new DSPatternHalfAndHalf(leds);

        // This shows how to change the pattern default colors.
        List<Integer> chaseColors = new ArrayList<Integer>();
        chaseColors.add(0, Color.BLACK);
        chaseColors.add(1, Color.GREEN);
        chase = new DSPatternChase(leds);
        chase.setPatternColors(chaseColors);

        // We wanted to try and use a level indicator to show kids their drive
        // speed.  A traditional "voltage" indicator would go red, yellow then
        // green.  In this case we wanted the mid value to be green to show
        // where we want them to be so we updated the default colors.
        indicator = new DSPatternLevelIndicator(leds);
        List<Integer> indicatorColors = new ArrayList<Integer>();
        indicatorColors.add(0, Color.YELLOW);
        indicatorColors.add(1, Color.GREEN);
        indicatorColors.add(2, Color.RED);
        indicator.setPatternColors(indicatorColors);

        // This creates a default workshop pattern
        workshop = new DSPatternWorkshop(leds);
    }

    public void start() {
        ledDisplay = rainbow;
        ledDisplay.update();
    }

    @Override
    public void loop() {
        try {
            if(gamepad1.a) {
                ledDisplay = twinkle;
            } else if(gamepad1.b) {
                ledDisplay = halfAndHalf;
            } else if(gamepad1.y) {
                ledDisplay = chase;
            } else if(gamepad1.x) {
                ledDisplay = indicator;
            } else if (gamepad1.left_bumper) {
                ledDisplay = rainbow;
            } else if (gamepad1.right_bumper) {
                ledDisplay = rainbowShift;
            } else if (gamepad1.dpad_up) {
                ledDisplay = workshop;
            }

            // The indicator pattern is the only one in this tester that
            // takes external input and puts it on the display.  In this
            // case it is the power applied by the joystick.
            double powerX = gamepad1.left_stick_x;
            double powerY = gamepad1.left_stick_y;
            double magnitude = Math.sqrt(powerX*powerX + powerY*powerY);
            ledDisplay.setMeasuredValue(magnitude);
            telemetry.addData("Joystick X", powerX);
            telemetry.addData("Joystick Y", powerY);
            telemetry.addData("Joystick Magnitude", magnitude);
            ledDisplay.update();
        } catch (Throwable ex) {
            telemetry.addData("Exception: ", ex.getMessage());
        }
    }

    @Override
    public void stop() {
    }
}

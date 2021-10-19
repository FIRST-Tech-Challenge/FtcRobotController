package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This sample illustrates using the rumble feature of many gamepads.
 *
 * Note: Some gamepads "rumble" better than others.
 *   The Xbox & PS4 have a left (rumble1) and right (rumble2) rumble motor.
 *   These two gamepads have two distinct rumble modes: Large on the left, and small on the right
 *   The ETpark gamepad may only respond to rumble1, and may only run at full power.
 *   The Logitech F310 gamepad does not have *any* rumble ability.
 *
 *   Moral:  You should use this sample to experiment with your specific gamepads to explore their rumble features.
 *
 * The rumble motors are accessed through the standard gamepad1 and gamepad2 objects.
 *   Several new methods were added to the Gamepad class in FTC SDK Rev 7
 *   The key methods are as follows:
 *
 *   .rumble(double rumble1, double rumble2, int durationMs)
 *     This method sets the rumble power of both motors for a specific time duration.
 *     Both rumble arguments are motor-power levels in the 0.0 to 1.0 range.
 *     durationMs is the desired length of the rumble action in milliseconds.
 *     This method returns immediately.
 *     Note:
 *       Use a durationMs of Gamepad.RUMBLE_DURATION_CONTINUOUS to provide a continuous rumble
 *       Use a power of 0, or duration of 0 to stop a rumble.
 *
 *   .rumbleBlips(int count) allows an easy way to signal the driver with a series of rumble blips.
 *     Just specify how many blips you want.
 *     This method returns immediately.
 *
 *   .runRumbleEffect(customRumbleEffect) allows you to run a custom rumble sequence that you have
 *     built using the Gamepad.RumbleEffect.Builder()
 *     A "custom effect" is a sequence of steps, where each step can rumble any of the
 *     rumble motors for a specific period at a specific power level.
 *     The Custom Effect will play as the (un-blocked) OpMode continues to run
 *
 *   .isRumbling() returns true if there is a rumble effect in progress.
 *     Use this call to prevent stepping on a current rumble.
 *
 *   .stopRumble()              Stop any ongoing rumble or custom rumble effect.
 *
 *   .rumble(int durationMs)    Full power rumble for fixed duration.
 *
 *   Note: Whenever a new Rumble command is issued, any currently executing rumble action will
 *   be truncated, and the new action started immediately.  Take these precautions:
 *      1) Do Not SPAM the rumble motors by issuing rapid fire commands
 *      2) Multiple sources for rumble commands must coordinate to avoid tromping on each other.
 *
 *   This can be achieved several possible ways:
 *   1) Only having one source for rumble actions
 *   2) Issuing rumble commands on transitions, rather than states.
 *      e.g. The moment a touch sensor is pressed, rather than the entire time it is being pressed.
 *   3) Scheduling rumble commands based on timed events. e.g. 10 seconds prior to endgame
 *   4) Rumble on non-overlapping mechanical actions. e.g. arm fully-extended or fully-retracted.
 *   5) Use isRumbling() to hold off on a new rumble if one is already in progress.
 *
 * The examples shown here are representstive of how to invoke a gamepad rumble.
 * It is assumed that these will be modified to suit the specific robot and team strategy needs.
 *
 * ########   Read the telemetry display on the Driver Station Screen for instructions.   ######
 *
 * Ex 1)    This example shows a) how to create a custom rumble effect, and then b) how to trigger it based
 *          on game time.  One use for this might be to alert the driver that half-time or End-game is approaching.
 *
 * Ex 2)    This example shows tying the rumble power to a changing sensor value.
 *          In this case it is the Gamepad trigger, but it could be any sensor output scaled to the 0-1 range.
 *          Since it takes over the rumble motors, it is only performed when the Left Bumper is pressed.
 *          Note that this approach MUST include a way to turn OFF the rumble when the button is released.
 *
 * Ex 3)    This example shows a simple way to trigger a 3-blip sequence.  In this case it is
 *          triggered by the gamepad A (Cross) button, but it could be any sensor, like a touch or light sensor.
 *          Note that this code ensures that it only rumbles once when the input goes true.
 *
 * Ex 4)    This example shows how to trigger a single rumble when an input value gets over a certain value.
 *          In this case it is reading the Right Trigger, but it could be any variable sensor, like a
 *          range sensor, or position sensor.  The code needs to ensure that it is only triggered once, so
 *          it waits till the sensor drops back below the threshold before it can trigger again.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */

@Disabled
@TeleOp(name="Concept: Gamepad Rumble", group ="Concept")
public class ConceptGamepadRumble extends LinearOpMode
{
    boolean lastA = false;                      // Use to track the prior button state.
    boolean lastLB = false;                     // Use to track the prior button state.
    boolean highLevel = false;                  // used to prevent multiple level-based rumbles.
    boolean secondHalf = false;                 // Use to prevent multiple half-time warning rumbles.

    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
    ElapsedTime runtime = new ElapsedTime();    // Use to determine when end game is starting.

    final double HALF_TIME = 60.0;              // Wait this many seconds before rumble-alert for half-time.
    final double TRIGGER_THRESHOLD  = 0.75;     // Squeeze more than 3/4 to get rumble.

    @Override
    public void runOpMode()
    {
        // Example 1. a)   start by creating a three-pulse rumble sequence: right, LEFT, LEFT
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .build();

        telemetry.addData(">", "Press Start");
        telemetry.update();

        waitForStart();
        runtime.reset();    // Start game timer.

        // Loop while monitoring buttons for rumble triggers
        while (opModeIsActive())
        {
            // Read and save the current gamepad button states.
            boolean currentA = gamepad1.a ;
            boolean currentLB = gamepad1.left_bumper ;

            // Display the current Rumble status.  Just for interest.
            telemetry.addData(">", "Are we RUMBLING? %s\n", gamepad1.isRumbling() ? "YES" : "no" );

            // ----------------------------------------------------------------------------------------
            // Example 1. b) Watch the runtime timer, and run the custom rumble when we hit half-time.
            //               Make sure we only signal once by setting "secondHalf" flag to prevent further rumbles.
            // ----------------------------------------------------------------------------------------
            if ((runtime.seconds() > HALF_TIME) && !secondHalf)  {
                gamepad1.runRumbleEffect(customRumbleEffect);
                secondHalf =true;
            }

            // Display the time remaining while we are still counting down.
            if (!secondHalf) {
                telemetry.addData(">", "Halftime Alert Countdown: %3.0f Sec \n", (HALF_TIME - runtime.seconds()) );
            }


            // ----------------------------------------------------------------------------------------
            // Example 2. If Left Bumper is being pressed, power the rumble motors based on the two trigger depressions.
            // This is useful to see how the rumble feels at various power levels.
            // ----------------------------------------------------------------------------------------
            if (currentLB) {
                // Left Bumper is being pressed, so send left and right "trigger" values to left and right rumble motors.
                gamepad1.rumble(gamepad1.left_trigger, gamepad1.right_trigger, Gamepad.RUMBLE_DURATION_CONTINUOUS);

                // Show what is being sent to rumbles
                telemetry.addData(">", "Squeeze triggers to control rumbles");
                telemetry.addData("> : Rumble", "Left: %.0f%%   Right: %.0f%%", gamepad1.left_trigger * 100, gamepad1.right_trigger * 100);
            } else {
                // Make sure rumble is turned off when Left Bumper is released (only one time each press)
                if (lastLB) {
                    gamepad1.stopRumble();
                }

                //  Prompt for manual rumble action
                telemetry.addData(">", "Hold Left-Bumper to test Manual Rumble");
                telemetry.addData(">", "Press A (Cross) for three blips");
                telemetry.addData(">", "Squeeze right trigger slowly for 1 blip");
            }
            lastLB = currentLB; // remember the current button state for next time around the loop


            // ----------------------------------------------------------------------------------------
            // Example 3. Blip 3 times at the moment that A (Cross) is pressed. (look for pressed transition)
            // BUT !!!  Skip it altogether if the Gamepad is already rumbling.
            // ----------------------------------------------------------------------------------------
            if (currentA && !lastA) {
                if (!gamepad1.isRumbling())  // Check for possible overlap of rumbles.
                    gamepad1.rumbleBlips(3);
            }
            lastA = currentA; // remember the current button state for next time around the loop


            // ----------------------------------------------------------------------------------------
            // Example 4. Rumble once when gamepad right trigger goes above the THRESHOLD.
            // ----------------------------------------------------------------------------------------
            if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                if (!highLevel) {
                    gamepad1.rumble(0.9, 0, 200);  // 200 mSec burst on left motor.
                    highLevel = true;  // Hold off any more triggers
                }
            } else {
                highLevel = false;  // We can trigger again now.
            }

            // Send the telemetry data to the Driver Station, and then pause to pace the program.
            telemetry.update();
            sleep(10);
        }
    }
}

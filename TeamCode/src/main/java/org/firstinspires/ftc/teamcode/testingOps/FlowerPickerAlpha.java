package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FlowerPickerSubsystem;

/*
 * Alpha-Flower Picker Testing (issue #75)
 * An iterative OpMode that moves the flower bar with the FlowerPickerSubsystem using gamepad1.
 * Use it to check that both servos move the right way, and to find the real UP and DOWN positions.
 *
 *   Right Bumper -> flower picker UP     (one move per press)
 *   Left Bumper  -> flower picker DOWN   (one move per press)
 *   D-pad Up / Down -> nudge the LEFT servo  +0.01 / -0.01 (each press)
 *   Y / A           -> nudge the RIGHT servo +0.01 / -0.01 (each press)
 *
 * If both bumpers go down on the same loop, UP wins (UP is the lifted, out-of-the-way place).
 *
 * THE TELEOP WILL USE THE SAME BUMPERS on gamepad2 (the operator). This OpMode uses gamepad1 like
 * the other test OpModes, so one person can test alone.
 *
 * THE NUDGE BUTTONS ARE FOR TESTING ONLY
 *   A nudge changes the number only while this OpMode runs. When you find a good number, type it
 *   into the constants at the top of FlowerPickerSubsystem (LEFT_UP_POSITION, LEFT_DOWN_POSITION,
 *   RIGHT_UP_POSITION, RIGHT_DOWN_POSITION). A nudge moves the CURRENT state's number: nudge while the
 *   bar is UP to tune UP, and nudge while it is DOWN to tune DOWN. The screen shows all four.
 *
 * SAFE FIRST TEST (do these in order):
 *   1. Robot on blocks, hands and cables clear of the bar. The numbers in the subsystem are STARTING
 *      GUESSES. At INIT both servos jump to the UP numbers, so expect a quick move.
 *   2. Press the Left Bumper (DOWN), then the Right Bumper (UP). Both sides must move together. If
 *      the bar moves the WRONG way, or only one side does, swap that side's UP and DOWN numbers in
 *      FlowerPickerSubsystem. If one side jams, press the other bumper right away.
 *   3. Use the nudge buttons to creep each side to the right spot, in small steps. Stop if a servo
 *      buzzes against a stop. That means the number is past the end of what the bar can do.
 *   4. Write down the four numbers from the screen, then type them into FlowerPickerSubsystem.
 *   5. Time one UP and one DOWN move with a phone stopwatch. If it takes more than
 *      SERVO_TRAVEL_SECONDS, make that number bigger.
 *
 * On a PS5 controller: the bumpers are L1 and R1, Y is Triangle and A is Cross.
 *
 * Robot configuration (on the Driver Station):
 *   Servo name: left_flower_servo
 *   Servo name: right_flower_servo
 *   Both are plain "Servo" devices in POSITIONAL mode.
 */
@TeleOp(name = "Flower Picker Testing - Alpha", group = "Testing")
public class FlowerPickerAlpha extends OpMode {

    // How much one nudge press changes a servo position. 0.01 is a small step (about 3 degrees on
    // a 300 degree servo). Make it smaller for a fine adjustment, bigger to get there faster.
    private static final double NUDGE_STEP = 0.01;

    // true = the control map stays on the Driver Station screen while the OpMode RUNS (at the
    // bottom, under the live data). false = it is only shown before START, like it used to be.
    private static final boolean SHOW_CONTROLS_WHILE_RUNNING = true;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private FlowerPickerSubsystem flowerPickerSubsystem;

    // What each button was doing LAST loop. A press lasts many loops, but we want it to count
    // only ONCE. So we only act when it was NOT pressed last loop and IS pressed now.
    private boolean wasRightBumperPressed = false;
    private boolean wasLeftBumperPressed = false;
    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;
    private boolean wasYPressed = false;
    private boolean wasAPressed = false;

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Right Bumper = picker UP   Left Bumper = picker DOWN");
        telemetry.addLine("D-pad Up / Down = nudge LEFT servo +/- " + NUDGE_STEP);
        telemetry.addLine("Y / A = nudge RIGHT servo +/- " + NUDGE_STEP);
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The subsystem gets its own servos from the hardware map, and moves the bar UP.
        flowerPickerSubsystem = new FlowerPickerSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        addControlsTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // The Driver Station clears its screen every loop, so we add the
        // button map again here. That keeps it on the screen until START.
        telemetry.addData("Status", "Ready to start!");
        addControlsTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Read the gamepad.
        boolean rightBumperPressed = gamepad1.right_bumper;
        boolean leftBumperPressed = gamepad1.left_bumper;
        boolean dpadUpPressed = gamepad1.dpad_up;
        boolean dpadDownPressed = gamepad1.dpad_down;
        boolean yPressed = gamepad1.y;
        boolean aPressed = gamepad1.a;

        // 2. Decide what to do. Each button counts only on the loop where it FIRST goes down.
        //    UP is checked first, so if both bumpers go down on the same loop, UP wins.
        if (rightBumperPressed && !wasRightBumperPressed) {
            flowerPickerSubsystem.goToUp();
        } else if (leftBumperPressed && !wasLeftBumperPressed) {
            flowerPickerSubsystem.goToDown();
        }

        if (dpadUpPressed && !wasDpadUpPressed) {
            flowerPickerSubsystem.nudgeLeftPosition(NUDGE_STEP);
        }
        if (dpadDownPressed && !wasDpadDownPressed) {
            flowerPickerSubsystem.nudgeLeftPosition(-NUDGE_STEP);
        }
        if (yPressed && !wasYPressed) {
            flowerPickerSubsystem.nudgeRightPosition(NUDGE_STEP);
        }
        if (aPressed && !wasAPressed) {
            flowerPickerSubsystem.nudgeRightPosition(-NUDGE_STEP);
        }

        // Remember the buttons for the next loop.
        wasRightBumperPressed = rightBumperPressed;
        wasLeftBumperPressed = leftBumperPressed;
        wasDpadUpPressed = dpadUpPressed;
        wasDpadDownPressed = dpadDownPressed;
        wasYPressed = yPressed;
        wasAPressed = aPressed;

        // 3. Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        flowerPickerSubsystem.addTelemetry(telemetry);

        // 4. Keep the button map on the screen while running, at the bottom under the live data.
        //    (Before START the same map is shown by init() and init_loop().)
        if (SHOW_CONTROLS_WHILE_RUNNING) {
            addControlsTelemetry();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // There is no motor to stop. The servos just stay where they were told to be.
    }
}

package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Defenition of the HzGamepad Class <BR>
 *
 * HzGamepad consists of system provided gamepad(s) and adds functionality to the selection 
 * made on gamepads <BR>
 * 
 * For Hazmat Skystone, only one Gamepad is used (gamepad1) <BR>
 *
 * The controls are as follows: <BR>
 *      <emsp>Left Stick for pan motion (gamepad1.left_stick_x and gamepad1.left_stick_y) <BR>
 *      <emsp>Right Stick for turn motion (only uses the x direction : gamepad1.right_stick_y) <BR>
 *      <emsp>Right Bumper for <TO-BE-UPDATED> (gamepad1.right_bumper) <BR>
 *      <emsp>Left Bumper for <TO-BE-UPDATED> (gamepad1.left_bumper) <BR>
 *      <emsp>Right Trigger for increasing speed to double (gamepad1.right_trigger) <BR>
 *      <emsp>Button A to <TO-BE-UPDATED> (gamepad1.a) <BR>
 *      <emsp>Button Y to <TO-BE-UPDATED> (gamepad1.y) <BR>
 *      <emsp>Button X to <TO-BE-UPDATED> (gamepad1.x) <BR>
 *      <emsp>Button B to <TO-BE-UPDATED> (gamepad1.b) <BR>
 *      <emsp>Button Dpad_up to <TO-BE-UPDATED> (gamepad1.dpad_up) <BR>
 *      <emsp>Button Dpad_down to <TO-BE-UPDATED> (gamepad1.dpad_down) <BR>
 */

public class HzGamepadClassic {

    //Create gamepad object reference to connect to gamepad1
    public Gamepad gpGamepad1;

    //Records last button press to deal with single button presses doing a certain methods
    boolean buttonALast = false;
    boolean buttonBLast = false;
    boolean buttonXLast = false;
    boolean buttonYLast = false;
    boolean rightBumperLast = false;
    boolean leftBumperLast = false;
    boolean dpad_upLast = false;
    boolean dpad_downLast = false;
    boolean gp1LeftTriggerLast = false;

    LinearOpMode opModepassed;
    /**
     * Constructor for HzGamepad1 class that extends gamepad.
     * Assign the gamepad1 given in OpMode to the gamepad used here.
     *
     * @param gamepadPassedfromOpMode from OpMode. In the case of Hazmat Skystone, this is gamepad1
     */
    public HzGamepadClassic(Gamepad gamepadPassedfromOpMode, LinearOpMode opModepassedfromOpMode) {
        gpGamepad1 = gamepadPassedfromOpMode;
        this.opModepassed = opModepassedfromOpMode;
    }

    /**
     * Methods to get the value of gamepad Left stick X for Pan motion X direction.
     * This is the method to apply any directional modifiers to match to the X plane of robot.
     * No modifier needed for Hazmat Skystone Robot.
     *
     * @return gpGamepad1.left_stick_x
     */
    public double getLeftStickX() {
        return gpGamepad1.left_stick_x;
    }

    /**
     * Methods to get the value of gamepad Left stick Y for Pan motion Y direction.
     * This is the method to apply any directional modifiers to match to the Y plane of robot.
     * For Hazmat Skystone Robot, Y direction needs to be inverted.
     *
     * @return gpGamepad1.left_stick_y * (-1)
     */
    public double getLeftStickY() {

        return gpGamepad1.left_stick_y * (-1);
         //return gpGamepad1.left_stick_y;
    }

    /**
     * Methods to get the value of gamepad Right stick X to keep turning.
     * This is the method to apply any directional modifiers to match to the turn direction robot.
     * No modifier needed for Hazmat Skystone Robot.
     *
     * @return gpGamepad1.right_stick_x
     */
    public double getRightStickX() {
        return gpGamepad1.right_stick_x;
    }

    /**
     * Methods to get the value of gamepad Right Trigger for turbo mode (max speed).
     * This is the method to apply any modifiers to match to action of turbo mode for each driver preference.
     * For Hazmat Skystone Right Trigger pressed means turbo mode on.
     *
     * @return gpGamepad1.right_trigger
     */
    public double getRightTrigger() {
        return gpGamepad1.right_trigger;
    }

    /**
     * Methods to get the value of gamepad Left Trigger for <TO-BE-UPDATED>
     *
     * @return gpGamepad1.right_trigger
     */
    public double getLeftTrigger() {
        return gpGamepad1.left_trigger;
    }

    public boolean getLeftTriggerPress() {
        boolean isPressedLeftTrigger = false;
        if (!gp1LeftTriggerLast && (getLeftTrigger()>0.95)) {
            isPressedLeftTrigger = true;
        }
        gp1LeftTriggerLast = (getLeftTrigger()>0.95) ? true : false;
        return isPressedLeftTrigger;
    }

    /**
     * Method to track if Left Bumper was pressed to <TO-BE-UPDATED>.
     * To ensure that the continuous holding of the left bumper does not cause a contiual action,
     * the state of the bumper is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing hold or release of button should not trigger action.
     *
     * @return isPressedLeftBumper| = true if prev state is not pressed and current is pressed.
     */
    public boolean getLeftBumperPress() {
        boolean isPressedLeftBumper = false;
        if (!leftBumperLast && gpGamepad1.left_bumper) {
            isPressedLeftBumper = true;
        }
        leftBumperLast = gpGamepad1.left_bumper;
        return isPressedLeftBumper;
    }

    /**
     * Method to track if Right Bumper was pressed to <TO-BE-UPDATED>.
     * To ensure that the continuous holding of the right bumper does not cause a continual action,
     * the state of the bumper is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedRightBumper = true if prev state is not pressed and current is pressed.
     */
    public boolean getRightBumperPress() {
        boolean isPressedRightBumper = false;
        if (!rightBumperLast && gpGamepad1.right_bumper) {
            isPressedRightBumper = true;
        }
        rightBumperLast = gpGamepad1.right_bumper;
        return isPressedRightBumper;
    }

    /**
     * Method to track if Button A was pressed to <TO-BE-UPDATED>.
     * To ensure that the continuous holding of Button A does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButton A = true if prev state is not pressed and current is pressed.
     */
    public boolean getButtonAPress() {
        boolean isPressedButtonA = false;
        if (!buttonALast && gpGamepad1.a) {
            isPressedButtonA = true;
        }
        buttonALast = gpGamepad1.a;
        return isPressedButtonA;
    }

    /**
     * Method to track if Button Y was pressed to <TO-BE-UPDATED>.
     * To ensure that the continuous holding of Button Y does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButtonY = true if prev state is not pressed and current is pressed.
     */
    public boolean getButtonYPress() {
        boolean isPressedButtonY = false;
        if (!buttonYLast && gpGamepad1.y) {
            isPressedButtonY = true;
        }
        buttonYLast = gpGamepad1.y;
        return isPressedButtonY;
    }

    /**
     * Method to track if Button X was pressed to <TO-BE-UPDATED>.
     * To ensure that the continuous holding of Button X does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButtonX = true if prev state is not pressed and current is pressed.
     */
    public boolean getButtonXPress() {
        boolean isPressedButtonX = false;
        if (!buttonXLast && gpGamepad1.x) {
            isPressedButtonX = true;
        }
        buttonXLast = gpGamepad1.x;
        return isPressedButtonX;
    }

    /**
     * Method to track if Button B was pressed to move Arm to <TO-BE-UPDATED>.
     * To ensure that the continuous holding of Button Y does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButtonB = true if prev state is not pressed and current is pressed.
     */
    public boolean getButtonBPress() {
        boolean isPressedButtonB = false;
        if (!buttonBLast && gpGamepad1.b) {
            isPressedButtonB = true;
        }
        buttonBLast = gpGamepad1.b;
        return isPressedButtonB;
    }

    /**
     * Method to track if Dpad_up was pressed to <TO-BE-UPDATED>.
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_up = true if prev state is not pressed and current is pressed.
     */
    public boolean getDpad_upPress() {
        boolean isPressedDpad_up;

        isPressedDpad_up = false;

        if (!dpad_upLast && gpGamepad1.dpad_up) {
            isPressedDpad_up = true;
        }
        dpad_upLast = gpGamepad1.dpad_up;
        return isPressedDpad_up;

    }

    public boolean getDpad_upPersistent(){
        return gpGamepad1.dpad_up;
    }

    /**
     * Method to track if Dpad_down was pressed to <TO-BE-UPDATED>.
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_down = true if prev state is not pressed and current is pressed.
     */
    public boolean getDpad_downPress() {
        boolean isPressedDpad_down;

        isPressedDpad_down = false;
        if (!dpad_downLast && gpGamepad1.dpad_down) {
            isPressedDpad_down = true;
        }
        dpad_downLast = gpGamepad1.dpad_down;
        return isPressedDpad_down;

    }

    /**
     * Method to convert linear map from gamepad1 stick input to a cubic map
     *
     * @param stickInput input value of button stick vector
     * @return Cube of the stick input reduced to 25% speed
     */
    public double limitStick(double stickInput) {
        return (stickInput * stickInput * stickInput * 0.25);
    }

    /**
     * Method to implement turbo speed mode - from reduced speed of 25% of cubic factor to
     * 100% speed, but controlled by acceleration of the force of pressing the Right Tigger.
     *
     * @param stickInput input value of button stick vector
     * @return modified value of button stick vector
     */
    public double turboMode(double stickInput) {

        double acceleration_factor;
        double rightTriggerValue;

        double turboFactor;

        rightTriggerValue = getRightTrigger();
        acceleration_factor = 1.0 + 3.0 * rightTriggerValue;
        turboFactor = limitStick(stickInput) * acceleration_factor;
        return turboFactor;
    }


    public void runByGamepadInputClassicChassis(HzChassisClassic gpChassis) {

        double leftStickX = turboMode(getLeftStickX());
        double leftStickY = turboMode(getLeftStickY());
        double rightStickX = turboMode(getRightStickX());
        double power = Math.hypot(leftStickX, leftStickY);
        double targetAngle = Math.atan2(leftStickY, leftStickX);
        double turn = rightStickX;
        gpChassis.runByGamepadCommand(targetAngle, turn, power);

        /*    if (getLeftTrigger()>0.5){}*/
        /*    if (getLeftBumperPress()) {}*/
        /*    if (getRightBumperPress()) {}*/
        /*    if (getButtonXPress()) {}*/
        /*    if (getButtonBPress()){}*/
        /*    if (getButtonAPress()){}*/
        /*    if (getButtonYPress()){}*/
        /*    if (getDpad_upPress()){}*/
        /*    if (getDpad_downPress()){}*/

    }

    public void runSubsystemByGamepadInput(HzChassisClassic gpChassis, HzArm gpHzArm, HzIntake gpHzIntake, HzLauncher gpHzLauncher, HzMagazine gpHzMagazine, HzLaunchController gpLauncherController) {

        //**** Chassis Actions****
        double leftStickX = turboMode(getLeftStickX());
        double leftStickY = turboMode(getLeftStickY());
        double rightStickX = turboMode(getRightStickX());
        double power = Math.hypot(leftStickX, leftStickY);
        double targetAngle = Math.atan2(leftStickY, leftStickX);
        double turn = rightStickX;
        gpChassis.runByGamepadCommand(targetAngle, turn, power);

        //runMagazineControl();
        //runIntakeControl();
        //runLaunchController();
        //runLauncher();
        //runArm();
    }
}


package org.firstinspires.ftc.teamcode.Controllers.Examples;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzArmUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.HzDrive;
import org.firstinspires.ftc.teamcode.GameOpModes.Examples.HzGameFieldUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzIntakeUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzLauncherUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzMagazineUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzVuforiaStatic;

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
 *      <emsp>Right Bumper for Launching Ring (gamepad1.right_bumper) <BR>
 *      <emsp>Left Bumper for Grip Arm Servos (gamepad1.left_bumper) <BR>
 *      <emsp>Right Trigger for Accelerating robot (gamepad1.right_trigger) <BR>
 *      <emsp>Button A to Powershot selection (gamepad1.a) <BR>
 *      <emsp>Button Y to High Goal selection (gamepad1.y) <BR>
 *      <emsp>Button X to Turn delta left (gamepad1.x) <BR>
 *      <emsp>Button B to Turn delta right (gamepad1.b) <BR>
 *      <emsp>Button Dpad_up to Reverse Intake on (gamepad1.dpad_up) <BR>
 *      <emsp>Button Dpad_down to Intake On (gamepad1.dpad_down) <BR>
 */

public class HzGamepadControllerUltimateGoal {

    //Create gamepad object reference to connect to gamepad1
    public Gamepad gpGamepad;
    public HzDrive gpDrive;
    public HzMagazineUltimateGoal gpHzMagazineUltimateGoal;
    public HzIntakeUltimateGoal gpHzIntakeUltimateGoal;
    public HzLaunchSubControllerUltimateGoal gpHzLaunchSubControllerUltimateGoal;
    public HzLauncherUltimateGoal gpHzLauncherUltimateGoal;
    public HzArmUltimateGoal gpHzArmUltimateGoal;

    /**
     * Constructor for HzGamepad1 class that extends gamepad.
     * Assign the gamepad1 given in OpMode to the gamepad used here.
     */
    public HzGamepadControllerUltimateGoal(Gamepad gamepadPassed,
                                           HzDrive gpDrivePassed,
                                           HzMagazineUltimateGoal gpHzMagazineUltimateGoalPassed,
                                           HzIntakeUltimateGoal gpHzIntakeUltimateGoalPassed,
                                           HzLaunchSubControllerUltimateGoal gpHzLaunchSubControllerUltimateGoalPassed,
                                           HzLauncherUltimateGoal gpHzLauncherUltimateGoalPassed,
                                           HzArmUltimateGoal gpHzArmUltimateGoalPassed) {
        gpGamepad = gamepadPassed;
        gpDrive = gpDrivePassed;
        gpHzMagazineUltimateGoal = gpHzMagazineUltimateGoalPassed;
        gpHzIntakeUltimateGoal = gpHzIntakeUltimateGoalPassed;
        gpHzLaunchSubControllerUltimateGoal = gpHzLaunchSubControllerUltimateGoalPassed;
        gpHzLauncherUltimateGoal = gpHzLauncherUltimateGoalPassed;
        gpHzArmUltimateGoal = gpHzArmUltimateGoalPassed;
        gpHzLaunchSubControllerUltimateGoal.batteryCorrectionFlag = true;
    }

    /**
     *runByGamepad is the main controller function that runs each subsystem controller based on states
     */
    public void runByGamepadControl(){
        runMagazineControl();
        runIntakeControl();
        runLaunchController();
        runLauncherControl();
        runDriveControl_byRRDriveModes();
        runArmControl();
    }

    /**
     * runByGamepadRRDriveModes sets modes for Road Runner such as ROBOT and FIELD Centric Modes. <BR>
     */
    // RR Drive Train
    public void runDriveControl_byRRDriveModes() {

        if (HzVuforiaStatic.vuforiaState == HzVuforiaStatic.VUFORIA_STATE.NAVIGATION_RUNNING &&
                HzVuforiaStatic.targetVisible){
            gpDrive.setPoseEstimate(HzVuforiaStatic.poseVuforia);
        }
        gpDrive.poseEstimate = gpDrive.getPoseEstimate();

        gpDrive.driveType = HzDrive.DriveType.ROBOT_CENTRIC;

        if (gpDrive.driveType == HzDrive.DriveType.ROBOT_CENTRIC){
            gpDrive.gamepadInput = new Vector2d(
                    -turboMode(getLeftStickY()) ,
                    -turboMode(getLeftStickX())
            );
        };

        if (gpDrive.driveType == HzDrive.DriveType.FIELD_CENTRIC){

            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.RED_ALLIANCE) { // Red Alliance
                gpDrive.gamepadInput = new Vector2d(
                        turboMode(getLeftStickX()),
                        -turboMode(getLeftStickY())
                ).rotated(-gpDrive.poseEstimate.getHeading());
            };

            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) { // Blue Alliance
                gpDrive.gamepadInput = new Vector2d(
                        -turboMode(getLeftStickX()),
                        turboMode(getLeftStickY())
                ).rotated(-gpDrive.poseEstimate.getHeading());
            };
        }
        gpDrive.gamepadInputTurn = -turboMode(getRightStickX());

        if (getButtonXPress()) {
            gpDrive.augmentedControl = HzDrive.AugmentedControl.TURN_DELTA_LEFT;
        }

        //Power Shot 2
        if (getButtonBPress()) {
            gpDrive.augmentedControl = HzDrive.AugmentedControl.TURN_DELTA_RIGHT;
        }

        gpDrive.driveTrainPointFieldModes();

    }

    /**
     * runMagazineControl sets 2 modes for Magazine, COLLECT and LAUNCH. <BR>
     */
    public void runMagazineControl(){ //this function should be in IntakeControl's place after order change
        if (gpHzMagazineUltimateGoal.magazinePosition == HzMagazineUltimateGoal.MAGAZINE_POSITION.AT_COLLECT){
            gpHzLauncherUltimateGoal.stopFlyWheel();
        }

        switch (gpHzMagazineUltimateGoal.moveMagazineTo) {
            case COLLECT:
                gpHzMagazineUltimateGoal.moveMagazineToCollect();
                break;
            case LAUNCH:
                gpHzMagazineUltimateGoal.moveMagazineToLaunch();
                break;
        }

    }

    /**
     * runIntakeControl sets the differnt intake controls, if intake should take in rings(Dpad_downPress) or the intake should run the opposite
     * direction in order for a stuck ring to be out of intake. <BR>
     */
    public void runIntakeControl(){ //this function should be at LaunchController's place after order change

        //Run Intake motors - start when Dpad_down is pressed once, and stop when it is pressed again
        if (getDpad_downPress()) {
            if (gpHzIntakeUltimateGoal.getIntakeState() != HzIntakeUltimateGoal.INTAKE_MOTOR_STATE.RUNNING) {
                gpHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = false;
                gpHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
                gpHzMagazineUltimateGoal.moveMagazineTo = HzMagazineUltimateGoal.MOVE_MAGAZINE_TO.COLLECT;
                gpHzIntakeUltimateGoal.intakeButtonState = HzIntakeUltimateGoal.INTAKE_BUTTON_STATE.ON;
                gpHzIntakeUltimateGoal.intakeReverseButtonState = HzIntakeUltimateGoal.INTAKE_REVERSE_BUTTON_STATE.OFF;
            } else if(gpHzIntakeUltimateGoal.getIntakeState() == HzIntakeUltimateGoal.INTAKE_MOTOR_STATE.RUNNING) {
                gpHzIntakeUltimateGoal.intakeButtonState = HzIntakeUltimateGoal.INTAKE_BUTTON_STATE.OFF;
            }
        }

        //Reverse Intake motors and run - in case of stuck state)
        //if (getDpad_upPersistent()) {
        if (getDpad_upPress()) {
            if (gpHzIntakeUltimateGoal.getIntakeState() != HzIntakeUltimateGoal.INTAKE_MOTOR_STATE.REVERSING){
                gpHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = false;
                gpHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;

                gpHzMagazineUltimateGoal.moveMagazineTo = HzMagazineUltimateGoal.MOVE_MAGAZINE_TO.COLLECT;
                gpHzIntakeUltimateGoal.intakeButtonState = HzIntakeUltimateGoal.INTAKE_BUTTON_STATE.OFF;
                gpHzIntakeUltimateGoal.intakeReverseButtonState = HzIntakeUltimateGoal.INTAKE_REVERSE_BUTTON_STATE.ON;
                //gpMagazine.shakeMagazine(100);
            } else if (gpHzIntakeUltimateGoal.getIntakeState() == HzIntakeUltimateGoal.INTAKE_MOTOR_STATE.REVERSING){
                gpHzIntakeUltimateGoal.intakeReverseButtonState = HzIntakeUltimateGoal.INTAKE_REVERSE_BUTTON_STATE.OFF;
            }
        }

        if (gpHzIntakeUltimateGoal.intakeButtonState == HzIntakeUltimateGoal.INTAKE_BUTTON_STATE.ON &&
                gpHzMagazineUltimateGoal.magazinePosition == HzMagazineUltimateGoal.MAGAZINE_POSITION.AT_COLLECT){
            gpHzIntakeUltimateGoal.runIntakeMotor();
        } else if (gpHzIntakeUltimateGoal.intakeButtonState == HzIntakeUltimateGoal.INTAKE_BUTTON_STATE.OFF &&
                gpHzIntakeUltimateGoal.getIntakeState() == HzIntakeUltimateGoal.INTAKE_MOTOR_STATE.RUNNING){
            gpHzIntakeUltimateGoal.stopIntakeMotor();
        }

        if (gpHzIntakeUltimateGoal.intakeReverseButtonState == HzIntakeUltimateGoal.INTAKE_REVERSE_BUTTON_STATE.ON) {
                gpHzIntakeUltimateGoal.reverseIntakeMotor();
        } else if ((gpHzIntakeUltimateGoal.intakeReverseButtonState == HzIntakeUltimateGoal.INTAKE_REVERSE_BUTTON_STATE.OFF &&
                gpHzIntakeUltimateGoal.getIntakeState() == HzIntakeUltimateGoal.INTAKE_MOTOR_STATE.REVERSING) ){
            gpHzIntakeUltimateGoal.stopIntakeMotor();
        }


    }

    /**
     * This function has the most basic controlller buttons, that sets the place each button should target. <BR>
     *     <emsp>StartPersistent and getButtonXPress decreases the velocity of each target by detecting the battery voltage
     *     and the current velocity power. </emsp> <BR>
     *     <emsp>StartPersistent and getButtonYPress increases the velocity of the flyhwheel by also detecting battery voltage. </emsp> <BR>
     *     <emsp>The ButtonY sets the target to High Goal and ButtonA Press sets the target to PowerShot 2.
     *     After this function, these two buttons are set to true to be executed. </emsp> <BR>
     */
    public void runLaunchController(){   //this function should be in magazineControl's place after order change
        /*if (getStartPersistent() && getButtonYPress()) {
            gpHzLaunchController.toggleModeManualAutomated();
        }*/

        if(getStartPersistent() && getButtonXPress()) {
            gpHzLauncherUltimateGoal.flyWheelVelocityHighGoal -= gpHzLauncherUltimateGoal.DELTA_VELOCITY_CORRECTION;
            gpHzLauncherUltimateGoal.flyWheelVelocityPowerShot -= gpHzLauncherUltimateGoal.DELTA_VELOCITY_CORRECTION;
        }

        if(getStartPersistent() && getButtonYPress()) {
            gpHzLauncherUltimateGoal.flyWheelVelocityHighGoal += gpHzLauncherUltimateGoal.DELTA_VELOCITY_CORRECTION;
            gpHzLauncherUltimateGoal.flyWheelVelocityPowerShot += gpHzLauncherUltimateGoal.DELTA_VELOCITY_CORRECTION;
        }

        if (gpHzLaunchSubControllerUltimateGoal.launchActivation == HzLaunchSubControllerUltimateGoal.LAUNCH_ACTIVATION.NOT_ACTIVATED) {
            //High, Middle, Low Goal
            if (getButtonYPress()) {
                gpHzLaunchSubControllerUltimateGoal.lcTarget = HzLaunchSubControllerUltimateGoal.LAUNCH_TARGET.HIGH_GOAL;
                gpHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = true;
            }


            //Power Shot 3
            if (getButtonAPress()) {
                gpHzLaunchSubControllerUltimateGoal.lcTarget = HzLaunchSubControllerUltimateGoal.LAUNCH_TARGET.POWER_SHOT2;
                gpHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = true;
            }
        }

        if (gpHzLaunchSubControllerUltimateGoal.launchActivation == HzLaunchSubControllerUltimateGoal.LAUNCH_ACTIVATION.ACTIVATED &&
                gpHzLaunchSubControllerUltimateGoal.launchMode == HzLaunchSubControllerUltimateGoal.LAUNCH_MODE.AUTOMATED) {
            gpHzLaunchSubControllerUltimateGoal.runLauncherByDistanceToTarget();
            if (getButtonYPress() || getButtonAPress()) {
                gpHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
            }
        }

        if (gpHzLaunchSubControllerUltimateGoal.launchActivation == HzLaunchSubControllerUltimateGoal.LAUNCH_ACTIVATION.ACTIVATED &&
                gpHzLaunchSubControllerUltimateGoal.launchMode == HzLaunchSubControllerUltimateGoal.LAUNCH_MODE.MANUAL) {
            if (gpHzLaunchSubControllerUltimateGoal.lcTarget == HzLaunchSubControllerUltimateGoal.LAUNCH_TARGET.HIGH_GOAL) {
                gpHzLauncherUltimateGoal.runFlyWheelToTarget(gpHzLauncherUltimateGoal.flyWheelVelocityHighGoal);
                if (getButtonYPress()) {
                    gpHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
                }
                if (getButtonAPress()) {
                    gpHzLaunchSubControllerUltimateGoal.lcTarget = HzLaunchSubControllerUltimateGoal.LAUNCH_TARGET.POWER_SHOT2;
                }
            } else { //gpHzLaunchController.lcTarget == HzLaunchController.LAUNCH_TARGET.POWER_SHOT2
                gpHzLauncherUltimateGoal.runFlyWheelToTarget(gpHzLauncherUltimateGoal.flyWheelVelocityPowerShot);
                if (getButtonAPress()) {
                    gpHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
                }
                if (getButtonYPress()) {
                    gpHzLaunchSubControllerUltimateGoal.lcTarget = HzLaunchSubControllerUltimateGoal.LAUNCH_TARGET.HIGH_GOAL;
                }
            }

            //if (getButtonYPress() || getButtonXPress() || getButtonBPress()|| getButtonAPress()) {
            /*if (getButtonYPress() || getButtonAPress()) {
                gpHzLaunchController.deactivateLaunchReadinessState = true;
            }*/
        }

        if (gpHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState) {
            gpHzLaunchSubControllerUltimateGoal.deactivateLaunchReadiness();
        }

        if (gpHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState) {
            gpHzLaunchSubControllerUltimateGoal.activateLaunchReadiness();
        }

    }

    /**
     * The runLauncher function tells if getRightBumperPress is set to true, and if it is, the Launcher plunges ring to Flywheel. <BR>
     *     Rapid fire if launch target is high goal & single shots if it is powershot
     */
    public void runLauncherControl(){
        if (gpHzLaunchSubControllerUltimateGoal.lcTarget == HzLaunchSubControllerUltimateGoal.LAUNCH_TARGET.HIGH_GOAL) {
            if (getRightBumperPersistant()) { //Rapid fire shots if bumper remains pressed
                //if (getRightBumperPress()) {
                if (gpHzLaunchSubControllerUltimateGoal.launchActivation == HzLaunchSubControllerUltimateGoal.LAUNCH_ACTIVATION.ACTIVATED &&
                        gpHzLaunchSubControllerUltimateGoal.launchReadiness == HzLaunchSubControllerUltimateGoal.LAUNCH_READINESS.READY) {
                    gpHzLaunchSubControllerUltimateGoal.setLaunchMotorVelocity();
                    gpHzLauncherUltimateGoal.plungeRingToFlyWheel();
                }
            }
        } else {
            if (getRightBumperPress()) { //Single shots on bumper press to ensure Power short is not rapid fire
                if (gpHzLaunchSubControllerUltimateGoal.launchActivation == HzLaunchSubControllerUltimateGoal.LAUNCH_ACTIVATION.ACTIVATED &&
                        gpHzLaunchSubControllerUltimateGoal.launchReadiness == HzLaunchSubControllerUltimateGoal.LAUNCH_READINESS.READY) {
                    gpHzLaunchSubControllerUltimateGoal.setLaunchMotorVelocity();
                    gpHzLauncherUltimateGoal.plungeRingToFlyWheel();
                }
            }
        }

    }


    /**
     * The leftTriggerPress when pressed once moves the arm halfway down, when it is pressed again the arm moves all the way down,
     * at last when the button is pressed for a third time the arm moves back to resting poistion. <BR>
     * The leftBumperPress when pressed once opens the arm, and when pressed again closes the arm grip. <BR>
     */
    public void runArmControl(){
        if (getLeftTriggerPress()) {
            gpHzArmUltimateGoal.moveArmByTrigger();
        }

        if (gpHzArmUltimateGoal.runArmToLevelState) {
            gpHzArmUltimateGoal.runArmToLevel(gpHzArmUltimateGoal.motorPowerToRun);
        }

        //Toggle Arm Grip actions
        if (getLeftBumperPress()) {
            if(gpHzArmUltimateGoal.getGripServoState() == HzArmUltimateGoal.GRIP_SERVO_STATE.OPENED) {
                gpHzArmUltimateGoal.closeGrip();
            } else if(gpHzArmUltimateGoal.getGripServoState() == HzArmUltimateGoal.GRIP_SERVO_STATE.CLOSED) {
                gpHzArmUltimateGoal.openGrip();
            }
        }
    }

    //*********** KEY PAD MODIFIERS BELOW ***********

    //**** Gamepad buttons
    //Records last button press to deal with single button presses doing a certain methods
    boolean gp1ButtonALast = false;
    boolean gp1ButtonBLast = false;
    boolean gp1ButtonXLast = false;
    boolean gp1ButtonYLast = false;
    boolean gp1RightBumperLast = false;
    boolean gp1LeftBumperLast = false;
    boolean gp1Dpad_upLast = false;
    boolean gp1Dpad_downLast = false;
    boolean gp1LeftTriggerLast = false;

    /**
     * Method to convert linear map from gamepad1 stick input to a cubic map
     *
     * @param stickInput input value of button stick vector
     * @return Cube of the stick input reduced to 25% speed
     */
    public double limitStick(double stickInput) {
        return (stickInput * stickInput * stickInput * 0.33);
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
        //acceleration_factor = 1.0 + 3.0 * rightTriggerValue;
        acceleration_factor = 1.0 + 2.0 * rightTriggerValue;
        turboFactor = limitStick(stickInput) * acceleration_factor;
        return turboFactor;
    }

    /**
     * Methods to get the value of gamepad Left stick X for Pan motion X direction.
     * This is the method to apply any directional modifiers to match to the X plane of robot.
     * No modifier needed for Hazmat Skystone Robot.
     *
     * @return gpGamepad1.left_stick_x
     */
    public double getLeftStickX() {
        return gpGamepad.left_stick_x;
    }

    /**
     * Methods to get the value of gamepad Left stick Y for Pan motion Y direction.
     * This is the method to apply any directional modifiers to match to the Y plane of robot.
     * For Hazmat Skystone Robot, Y direction needs to be inverted.
     *
     * @return gpGamepad1.left_stick_y
     */
    public double getLeftStickY() { return gpGamepad.left_stick_y; }

    /**
     * Methods to get the value of gamepad Right stick X to keep turning.
     * This is the method to apply any directional modifiers to match to the turn direction robot.
     * No modifier needed for Hazmat Skystone Robot.
     *
     * @return gpGamepad1.right_stick_x
     */
    public double getRightStickX() {
        return gpGamepad.right_stick_x;
    }

    /**
     * Methods to get the value of gamepad Right Trigger for turbo mode (max speed).
     * This is the method to apply any modifiers to match to action of turbo mode for each driver preference.
     * For Hazmat Skystone Right Trigger pressed means turbo mode on.
     *
     * @return gpGamepad1.right_trigger
     */
    public double getRightTrigger() {
        return gpGamepad.right_trigger;
    }

    /**
     * Methods to get the value of gamepad Left Trigger
     *
     * @return gpGamepad1.right_trigger
     */
    public double getLeftTrigger() {
        return gpGamepad.left_trigger;
    }

    public boolean getLeftTriggerPress() {
        boolean isPressedLeftTrigger = false;
        if (!gp1LeftTriggerLast && (getLeftTrigger()>0.7)) {
            isPressedLeftTrigger = true;
        }
        gp1LeftTriggerLast = (getLeftTrigger()>0.7);
        return isPressedLeftTrigger;
    }

    /**
     * Method to track if Left Bumper was pressed
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
        if (!gp1LeftBumperLast && gpGamepad.left_bumper) {
            isPressedLeftBumper = true;
        }
        gp1LeftBumperLast = gpGamepad.left_bumper;
        return isPressedLeftBumper;
    }


    /**
     * Method to track if Right Bumper was pressed
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
        if (!gp1RightBumperLast && gpGamepad.right_bumper) {
            isPressedRightBumper = true;
        }
        gp1RightBumperLast = gpGamepad.right_bumper;
        return isPressedRightBumper;
    }

    public boolean getRightBumperPersistant(){
        return gpGamepad.right_bumper;
    }

    /**
     * Method to track if Button A was pressed
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
        if (!gp1ButtonALast && gpGamepad.a) {
            isPressedButtonA = true;
        }
        gp1ButtonALast = gpGamepad.a;
        return isPressedButtonA;
    }

    /**
     * Method to track if Button Y was pressed
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
        if (!gp1ButtonYLast && gpGamepad.y) {
            isPressedButtonY = true;
        }
        gp1ButtonYLast = gpGamepad.y;
        return isPressedButtonY;
    }

    /**
     * Method to track if Button X was pressed
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
        if (!gp1ButtonXLast && gpGamepad.x) {
            isPressedButtonX = true;
        }
        gp1ButtonXLast = gpGamepad.x;
        return isPressedButtonX;
    }

    /**
     * Method to track if Button B was pressed to move Arm
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
        if (!gp1ButtonBLast && gpGamepad.b) {
            isPressedButtonB = true;
        }
        gp1ButtonBLast = gpGamepad.b;
        return isPressedButtonB;
    }

    /**
     * Method to track if Dpad_up was pressed
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

        if (!gp1Dpad_upLast && gpGamepad.dpad_up) {
            isPressedDpad_up = true;
        }
        gp1Dpad_upLast = gpGamepad.dpad_up;
        return isPressedDpad_up;

    }

    public boolean getDpad_upPersistent(){
        return gpGamepad.dpad_up;
    }

    /**
     * Method to track if Dpad_down was pressed
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
        if (!gp1Dpad_downLast && gpGamepad.dpad_down) {
            isPressedDpad_down = true;
        }
        gp1Dpad_downLast = gpGamepad.dpad_down;
        return isPressedDpad_down;

    }

    public boolean getStartPersistent(){
        return gpGamepad.start;
    }

}
package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;
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

public class HzGamepad {

    //Create gamepad object reference to connect to gamepad1
    public Gamepad gpGamepad;
    public HzDrive gpDrive;
    public Magazine gpMagazine;
    public Intake gpIntake;
    public LaunchController gpLaunchController;
    public Launcher gpLauncher;
    public Arm gpArm;

    /**
     * Constructor for HzGamepad1 class that extends gamepad.
     * Assign the gamepad1 given in OpMode to the gamepad used here.
     *
     * @param gamepadPassed from OpMode. In the case of Hazmat Skystone, this is gamepad1
     */
    public HzGamepad(Gamepad gamepadPassed,
                     HzDrive gpDrivePassed,
                     Magazine gpMagazinePassed,
                     Intake gpIntakePassed,
                     LaunchController gpLaunchControllerPassed,
                     Launcher gpLauncherPassed,
                     Arm gpArmPassed) {
        gpGamepad = gamepadPassed;
        gpDrive = gpDrivePassed;
        gpMagazine = gpMagazinePassed;
        gpIntake = gpIntakePassed;
        gpLaunchController = gpLaunchControllerPassed;
        gpLauncher = gpLauncherPassed;
        gpArm = gpArmPassed;
    }

    public void runByGamepad(){
        runMagazineControl();
        runIntakeControl();
        runLaunchController();
        runLauncher();
        runByGamepadRRDriveModes();
        runArm();
    }


    // RR Drive Train
    public void runByGamepadRRDriveModes(/*HzDrive gpDrive, int playingAlliance*/) {
        gpDrive.poseEstimate = gpDrive.getPoseEstimate();

        gpDrive.driveType = HzDrive.DriveType.FIELD_CENTRIC;

        if (gpDrive.driveType == HzDrive.DriveType.ROBOT_CENTRIC){
            gpDrive.gamepadInput = new Vector2d(
                    -turboMode(getLeftStickY()) ,
                    -turboMode(getLeftStickX())
            );//.rotated(-gpDrive.poseEstimate.getHeading());
        };

        if (gpDrive.driveType == HzDrive.DriveType.FIELD_CENTRIC){
            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.AUDIENCE) { // Audience
                gpDrive.gamepadInput = new Vector2d(
                        -turboMode(getLeftStickY()),
                        -turboMode(getLeftStickX())
                ).rotated(-gpDrive.poseEstimate.getHeading());
            }

            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) { // Red Alliance
                gpDrive.gamepadInput = new Vector2d(
                        turboMode(getLeftStickX()),
                        -turboMode(getLeftStickY())
                ).rotated(-gpDrive.poseEstimate.getHeading());
            };

            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) { // Blue Alliance
                gpDrive.gamepadInput = new Vector2d(
                        -turboMode(getLeftStickX()),
                        turboMode(getLeftStickY())
                ).rotated(-gpDrive.poseEstimate.getHeading());
            };
        }
        gpDrive.gamepadInputTurn = -turboMode(getRightStickX());

        //TODO : AMJAD : LaunchController to be invoked here to invoke drive mode and point to align
        //drivePointToAlign = Target Vector;
        //drivePointToAlign = BLUE_TOWER_GOAL;

        //gpDrive.driveTrainFieldCentric();
        gpDrive.driveTrainPointFieldModes();

    }

    public void runMagazineControl(){
        if (gpMagazine.magazinePosition == Magazine.MAGAZINE_POSITION.AT_COLLECT){
            gpLauncher.stopFlyWheel();
        }

        if (gpMagazine.moveMagazineToCollectState) {
            gpMagazine.moveMagazineToCollect();
        }

        if (gpMagazine.moveMagazineToLaunchState){
            gpMagazine.moveMagazineToLaunch();
        }

    }


    public void runIntakeControl(){

        //Run Intake motors - start when Dpad_down is pressed once, and stop when it is pressed again
        if (getDpad_downPress()) {
            if (gpIntake.getIntakeState() == Intake.INTAKE_MOTOR_STATE.STOPPED) {
                gpLaunchController.activateLaunchReadinessState = false;
                gpLaunchController.deactivateLaunchReadinessState = true;
                gpMagazine.moveMagazineToCollectState = true;
                gpIntake.intakeButtonState = Intake.INTAKE_BUTTON_STATE.ON;
            } else if(gpIntake.getIntakeState() == Intake.INTAKE_MOTOR_STATE.RUNNING) {
                gpIntake.intakeButtonState = Intake.INTAKE_BUTTON_STATE.OFF;
            }
        }

        if (gpIntake.intakeButtonState == Intake.INTAKE_BUTTON_STATE.ON &&
                gpMagazine.magazinePosition == Magazine.MAGAZINE_POSITION.AT_COLLECT){
            gpIntake.runIntakeMotor();
        } else {
            gpIntake.stopIntakeMotor();
        }

        //Reverse Intake motors and run - in case of stuck state)
        if (getDpad_upPersistent()) {
            gpIntake.intakeButtonState = Intake.INTAKE_BUTTON_STATE.OFF;
            gpIntake.reverseIntakeMotor();
            //gpMagazine.shakeMagazine(100);
        } else if (gpIntake.getIntakeState() == Intake.INTAKE_MOTOR_STATE.REVERSING){
            gpIntake.stopIntakeMotor();
        }
    }


    public void runLaunchController(){
        if (getStartPersistent() && getButtonYPress()) {
            gpLaunchController.toggleModeManualAutomated();
        }

        if (gpLaunchController.launchActivation == LaunchController.LAUNCH_ACTIVATION.NOT_ACTIVATED) {
            //High, Middle, Low Goal
            if (getButtonYPress()) {
                gpLaunchController.lcTarget = LaunchController.LAUNCH_TARGET.HIGH_GOAL;
                gpLaunchController.activateLaunchReadinessState = true;
            }

            //Power Shot 1
            if (getButtonXPress()) {
                gpLaunchController.lcTarget = LaunchController.LAUNCH_TARGET.POWER_SHOT1;
                gpLaunchController.activateLaunchReadinessState = true;
            }

            //Power Shot 2
            if (getButtonBPress()) {
                gpLaunchController.lcTarget = LaunchController.LAUNCH_TARGET.POWER_SHOT2;
                gpLaunchController.activateLaunchReadinessState = true;
            }

            //Power Shot 3
            if (getButtonAPress()) {
                gpLaunchController.lcTarget = LaunchController.LAUNCH_TARGET.POWER_SHOT3;
                gpLaunchController.activateLaunchReadinessState = true;
            }
        }

        if (gpLaunchController.launchActivation == LaunchController.LAUNCH_ACTIVATION.ACTIVATED) {
            gpLaunchController.runLauncherByDistanceToTarget();
            if (getButtonYPress() || getButtonXPress() || getButtonBPress()|| getButtonAPress()) {
                gpLaunchController.deactivateLaunchReadinessState = true;
            }
        }

        if (gpLaunchController.deactivateLaunchReadinessState) {
            gpLaunchController.deactivateLaunchReadiness();
        }

        if (gpLaunchController.activateLaunchReadinessState) {
            gpLaunchController.activateLaunchReadiness();
        }

        gpLaunchController.indicateLaunchReadiness();

    }

    public void runLauncher(){
        if (getRightBumperPress()) {
            if (gpLaunchController.launchActivation == LaunchController.LAUNCH_ACTIVATION.ACTIVATED &&
                    gpLaunchController.launchReadiness == LaunchController.LAUNCH_READINESS.READY) {
                gpLauncher.plungeRingToFlyWheel();
            }
        }
    }

    public void runArm(){
        if (getLeftTriggerPress()) {
            gpArm.moveArmByTrigger();
        }

        if (gpArm.runArmToLevelState) {
            gpArm.runArmToLevel(gpArm.motorPowerToRun);
        }

        //Toggle Arm Grip actions
        if (getLeftBumperPress()) {
            if(gpArm.getGripServoState() == Arm.GRIP_SERVO_STATE.OPENED) {
                gpArm.closeGrip();
            } else if(gpArm.getGripServoState() == Arm.GRIP_SERVO_STATE.CLOSED) {
                gpArm.openGrip();
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
     * Methods to get the value of gamepad Left Trigger for <TO-BE-UPDATED>
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
        if (!gp1LeftBumperLast && gpGamepad.left_bumper) {
            isPressedLeftBumper = true;
        }
        gp1LeftBumperLast = gpGamepad.left_bumper;
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
        if (!gp1RightBumperLast && gpGamepad.right_bumper) {
            isPressedRightBumper = true;
        }
        gp1RightBumperLast = gpGamepad.right_bumper;
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
        if (!gp1ButtonALast && gpGamepad.a) {
            isPressedButtonA = true;
        }
        gp1ButtonALast = gpGamepad.a;
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
        if (!gp1ButtonYLast && gpGamepad.y) {
            isPressedButtonY = true;
        }
        gp1ButtonYLast = gpGamepad.y;
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
        if (!gp1ButtonXLast && gpGamepad.x) {
            isPressedButtonX = true;
        }
        gp1ButtonXLast = gpGamepad.x;
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
        if (!gp1ButtonBLast && gpGamepad.b) {
            isPressedButtonB = true;
        }
        gp1ButtonBLast = gpGamepad.b;
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
        if (!gp1Dpad_downLast && gpGamepad.dpad_down) {
            isPressedDpad_down = true;
        }
        gp1Dpad_downLast = gpGamepad.dpad_down;
        return isPressedDpad_down;

    }

    public boolean getStartPersistent(){
        return gpGamepad.start;
    }



/*
    Scale a number in the range of x1 to x2, to the range of y1 to y2
    Range.scale(double n, double x1, double x2, double y1, double y2)

    Example for using Range.clip and scaleInput to switch to array points
    public void loop() {

        double rightDriveMotorsSpeed = -gamepad1.right_stick_y;
        double leftDriveMotorsSpeed  = -gamepad1.left_stick_y;

        rightDriveMotorsSpeed = Range.clip(rightDriveMotorsSpeed, -1, 1);
        leftDriveMotorsSpeed = Range.clip(leftDriveMotorsSpeed, -1, 1);

        rightDriveMotorsSpeed = scaleInput(rightDriveMotorsSpeed);
        leftDriveMotorsSpeed = scaleInput(leftDriveMotorsSpeed);

        driveRightFront.setPower(rightDriveMotorsSpeed);
        driveRightBack.setPower(rightDriveMotorsSpeed);
        driveLeftFront.setPower(leftDriveMotorsSpeed);
        driveLeftBack.setPower(leftDriveMotorsSpeed);

    }

    @Override
    public void stop() {
    }

    //Scales Motor Power
    double scaleInput(double Val) {

        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24, 0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (Val * 16.0);

        // index should be positive.
        if(index < 0){
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if(index > 16){
            index = 16;
        }

        // get value from the array.
        double Scale = 0.0;
        if(Val < 0){
            Scale = scaleArray[index];
        } else {
            Scale = -scaleArray[index];
        }

        // return scaled value.
        return Scale;
    }*/


}
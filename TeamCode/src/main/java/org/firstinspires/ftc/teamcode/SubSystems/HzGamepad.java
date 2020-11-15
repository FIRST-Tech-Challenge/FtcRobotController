package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

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
    public Gamepad gpGamepad1;

    //**** Drive Train ****
    //SampleMecanumDrive gpDrive;
    //For Position
    public Pose2d poseEstimate;

    //**** Align to point and Field Drive Mode ****
    // Define 2 states, driver control or alignment control
    enum DriveMode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT;
        DriveMode toggle() {
            if (this.equals(NORMAL_CONTROL))
                return ALIGN_TO_POINT;
            else
                return NORMAL_CONTROL;
        }
    }

    public DriveMode driveMode = DriveMode.NORMAL_CONTROL; //Default initializer
    public static double DRAWING_TARGET_RADIUS = 2;

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    public static final Vector2d origin = new Vector2d(0,0);
    public static final Vector2d blueGoal = new Vector2d(72,36);
    public static final Vector2d redGoal = new Vector2d(72,-36);
    public static final Vector2d bluePowerShot1 = new Vector2d(72,36);
    public static final Vector2d bluePowerShot2 = new Vector2d(72,36);
    public static final Vector2d bluePowerShot3 = new Vector2d(72,36);
    public static final Vector2d redPowerShot1 = new Vector2d(72,-36);
    public static final Vector2d redPowerShot2 = new Vector2d(72,-36);
    public static final Vector2d redPowerShot3 = new Vector2d(72,-36);

    private Vector2d drivePointToAlign = origin;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);


    //**** Gamepad buttons
    //Records last button press to deal with single button presses doing a certain methods
    boolean buttonALast = false;
    boolean buttonBLast = false;
    boolean buttonXLast = false;
    boolean buttonYLast = false;
    boolean rightBumperLast = false;
    boolean leftBumperLast = false;
    boolean dpad_upLast = false;
    boolean dpad_downLast = false;

    /**
     * Constructor for HzGamepad1 class that extends gamepad.
     * Assign the gamepad1 given in OpMode to the gamepad used here.
     *
     * @param gamepadPassedfromOpMode from OpMode. In the case of Hazmat Skystone, this is gamepad1
     */
    public HzGamepad(Gamepad gamepadPassedfromOpMode) {
        gpGamepad1 = gamepadPassedfromOpMode;
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


    // Classic Drive Train
    public void runByGamepadInputClassicChassis(ChassisClassic gpChassisClassic) {

        /*    if(getLeftTrigger()>0.5){}*/
        /*    if (getLeftBumperPress()) {}*/
        /*    if (getRightBumperPress()) {}*/
        /*    if (getButtonXPress()) {}*/
        /*    if (getButtonBPress()){}*/
        /*    if (getButtonAPress()){}*/
        /*    if (getButtonYPress()){}*/
        /*    if (getDpad_upPress()){}*/
        /*    if (getDpad_downPress()){}*/

        //Run Classic DriveTrain
        double leftStickX = turboMode(getLeftStickX());
        double leftStickY = turboMode(getLeftStickY());
        double rightStickX = turboMode(getRightStickX());
        double power = Math.hypot(leftStickX, leftStickY);
        double targetAngle = Math.atan2(leftStickY, leftStickX);
        double turn = rightStickX;
        gpChassisClassic.runByGamepadCommand(targetAngle, turn, power);
    }


    // RR Drive Train
    public void runByGamepadRRDriveModes(LinearOpMode callingOpMode, SampleMecanumDrive gpDrive, int playingAlliance) {
        //this.gpDrive = driveHandlePassedFromOpMode;

        /*    if(getLeftTrigger()>0.5){}*/
        /*    if (getLeftBumperPress()) {}*/
        /*    if (getRightBumperPress()) {}*/
        /*    if (getButtonXPress()) {}*/
        /*    if (getButtonBPress()){}*/
        /*    if (getButtonAPress()){}*/
        /*    if (getButtonYPress()){}*/
        /*    if (getDpad_upPress()){}*/
        /*    if (getDpad_downPress()){}*/

        //Code to toggle Drive Mode when Y is pressed
        if (getButtonYPress()){ driveMode.toggle();}

        //driveTrainFieldCentric(gpDrive);

        //drivePointToAlign = Target Vector;
        driveTrainPointFieldModes(/*callingOpMode*/ gpDrive, drivePointToAlign);

    }

    public void driveTrainFieldCentric(SampleMecanumDrive gpDrive){

        poseEstimate = gpDrive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -turboMode(getLeftStickY()) /* TODO : playingalliance modifier*/,
                -turboMode(getLeftStickX()) /* TODO : playingalliance modifier*/
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        gpDrive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -turboMode(getRightStickX()) /* TODO : playingalliance modifier*/
                )
        );

        // Update everything. Odometry. Etc.
        gpDrive.update();
    }


    public void driveTrainPointFieldModes(/*LinearOpMode callingOpMode*/ SampleMecanumDrive gpDrive, Vector2d pointToAlign){
        poseEstimate = gpDrive.getPoseEstimate();

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

        // Declare a drive direction
        // Pose representing desired x, y, and angular velocity
        Pose2d driveDirection = new Pose2d();

        //callingOpMode.telemetry.addData("mode", driveMode);

        // Declare telemetry packet for dashboard field drawing
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        switch (driveMode) {
            case NORMAL_CONTROL:
                // Switch into alignment mode if `a` is pressed
                /*if (gamepad1.a) {*/
                    driveMode = driveMode.ALIGN_TO_POINT;
                /*}*/

                // Standard teleop control
                // Convert gamepad input into desired pose velocity
                driveDirection = new Pose2d(
                        -turboMode(getLeftStickY()),
                        -turboMode(getLeftStickX()),
                        -turboMode(getRightStickX())
                );
                break;
            case ALIGN_TO_POINT:
                // Switch back into normal driver control mode if `b` is pressed
                /*if (gamepad1.b) {*/
                    driveMode = driveMode.NORMAL_CONTROL;
                /*}*/

                // Create a vector from the gamepad x/y inputs which is the field relative movement
                // Then, rotate that vector by the inverse of that heading for field centric control
                Vector2d fieldFrameInput = new Vector2d(
                        -turboMode(getLeftStickY()),
                        -turboMode(getLeftStickX())
                );
                Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                // Difference between the target vector and the bot's position
                Vector2d difference = drivePointToAlign.minus(poseEstimate.vec());
                // Obtain the target angle for feedback and derivative for feedforward
                double theta = difference.angle();

                // Not technically omega because its power. This is the derivative of atan2
                double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                // Set the target heading for the heading controller to our desired angle
                headingController.setTargetPosition(theta);

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(poseEstimate.getHeading())
                        * DriveConstants.kV + thetaFF)
                        * DriveConstants.TRACK_WIDTH;

                // Combine the field centric x/y velocity with our derived angular velocity
                driveDirection = new Pose2d(
                        robotFrameInput,
                        headingInput
                );

                // Draw the target on the field
                fieldOverlay.setStroke("#dd2c00");
                fieldOverlay.strokeCircle(drivePointToAlign.getX(), drivePointToAlign.getY(), DRAWING_TARGET_RADIUS);

                // Draw lines to target
                fieldOverlay.setStroke("#b89eff");
                fieldOverlay.strokeLine(drivePointToAlign.getX(), drivePointToAlign.getY(), poseEstimate.getX(), poseEstimate.getY());
                fieldOverlay.setStroke("#ffce7a");
                fieldOverlay.strokeLine(drivePointToAlign.getX(), drivePointToAlign.getY(), drivePointToAlign.getX(), poseEstimate.getY());
                fieldOverlay.strokeLine(drivePointToAlign.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());
                break;
        }

        // Draw bot on canvas
        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

        gpDrive.setWeightedDrivePower(driveDirection);

        // Update the heading controller with our current heading
        headingController.update(poseEstimate.getHeading());

        // Update he localizer
        gpDrive.getLocalizer().update();

        // Send telemetry packet off to dashboard
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        // Print pose to telemetry
        //telemetry.addData("x", poseEstimate.getX());
        //telemetry.addData("y", poseEstimate.getY());
        //telemetry.addData("heading", poseEstimate.getHeading());
        //telemetry.update();
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
    public double getLeftStickY() { return gpGamepad1.left_stick_y * (-1); }

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
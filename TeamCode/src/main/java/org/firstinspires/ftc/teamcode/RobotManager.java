/* Authors: Arin Khare, Kai Vernooy
 */

package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;


/** A completely encompassing class of all functionality of the robot. An OpMode should interface through an instance of
 *  this class in order to send or receive any data with the real robot.
 */
public class RobotManager {

    static final double JOYSTICK_DEAD_ZONE_SIZE = 0.05;
    static final double TRIGGER_DEAD_ZONE_SIZE = 0.05;

    public enum AllianceColor {BLUE, RED}
    public enum StartingSide {OUR_COLOR, THEIR_COLOR} //add starting side here later
    public Robot robot;
    public AllianceColor allianceColor;
    public StartingSide startingSide;

    public MechanismDriving mechanismDriving;
    public Navigation navigation;
    public ComputerVision computerVision;

    protected GamepadWrapper gamepads, previousStateGamepads;

    private Telemetry telemetry;
    public ElapsedTime elapsedTime;

    public RobotManager(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2,
                        ArrayList<Position> path, AllianceColor allianceColor, StartingSide startingSide,
                        Navigation.MovementMode movementMode, Telemetry telemetry, ElapsedTime elapsedTime) {

        this.telemetry = telemetry;
        this.elapsedTime = elapsedTime;
        this.allianceColor = allianceColor;
        this.startingSide = startingSide;

        elapsedTime.reset();
        navigation = new Navigation(path, allianceColor, startingSide, movementMode);
        mechanismDriving = new MechanismDriving();

        robot = new Robot(hardwareMap, telemetry, elapsedTime);

//        if (!path.isEmpty()) {
//            computerVision = new ComputerVision(hardwareMap, new AutonPipeline(robot, telemetry, allianceColor));
//        }

        gamepads = new GamepadWrapper(gamepad1, gamepad2);
        previousStateGamepads = new GamepadWrapper();
        previousStateGamepads.copyGamepads(gamepads);
    }

    // TELE-OP
    // =======

    /** Determine new robot desired states based on controller input (checks for button releases)
     */
    public void readControllerInputs() {
        // Linear slides
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_RETRACTED)) {
            if (robot.desiredHorseshoeState == Robot.HorseshoeState.FRONT)
                Robot.desiredSlidesState = Robot.SlidesState.RETRACTED;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_LOW)) {
            Robot.desiredSlidesState = Robot.SlidesState.LOW;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_MEDIUM)) {
            Robot.desiredSlidesState = Robot.SlidesState.MEDIUM;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_HIGH)) {
            Robot.desiredSlidesState = Robot.SlidesState.HIGH;
        }
        if (gamepads.getAnalogValues().gamepad2LeftStickY > RobotManager.JOYSTICK_DEAD_ZONE_SIZE) {
            Robot.desiredSlidesState = Robot.SlidesState.MOVE_UP;
        }
        if (gamepads.getAnalogValues().gamepad2LeftStickY < RobotManager.JOYSTICK_DEAD_ZONE_SIZE) {
            Robot.desiredSlidesState = Robot.SlidesState.MOVE_DOWN;
        }

        //if (getButtonRelease(GamepadWrapper.DriverAction.TOGGLE_WHEEL_SPEED_ADJUSTMENT)) {
        //    robot.wheelSpeedAdjustment = !robot.wheelSpeedAdjustment;
        //}

        //Horseshoe movement FRONT
        if(getButtonRelease(GamepadWrapper.DriverAction.HORSESHOE_TO_FRONT)){
            robot.desiredHorseshoeState = Robot.HorseshoeState.FRONT;
        }
        //Horseshoe movement BACK
        if(getButtonRelease(GamepadWrapper.DriverAction.HORSESHOE_TO_BACK)){
            if(Robot.desiredSlidesState!=Robot.SlidesState.RETRACTED)
                robot.desiredHorseshoeState = Robot.HorseshoeState.REAR; //Rear is back.
        }

        // Claw
        if (getButtonRelease(GamepadWrapper.DriverAction.OPEN_CLAW)) {
            robot.desiredClawState = Robot.ClawState.OPEN;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.CLOSE_CLAW)) {
            robot.desiredClawState = Robot.ClawState.CLOSED;
        }

        // Adjust relative wheel speeds.

        if (robot.wheelSpeedAdjustment) {
            // Left stick Y for adjusting rear left.
            if (gamepads.getAnalogValues().gamepad2LeftStickY > 0.5) {
                navigation.wheel_speeds[0] += 0.01;
            }
            if (gamepads.getAnalogValues().gamepad2LeftStickY < -0.5) {
                navigation.wheel_speeds[0] -= 0.01;
            }
            // Left stick X for adjusting rear right.
            if (gamepads.getAnalogValues().gamepad2LeftStickX > 0.5) {
                navigation.wheel_speeds[1] += 0.01;
            }
            if (gamepads.getAnalogValues().gamepad2LeftStickX < -0.5) {
                navigation.wheel_speeds[1] -= 0.01;
            }
            // Right stick Y for adjusting front left.
            if (gamepads.getAnalogValues().gamepad2RightStickY > 0.5) {
                navigation.wheel_speeds[2] += 0.01;
            }
            if (gamepads.getAnalogValues().gamepad2RightStickY < -0.5) {
                navigation.wheel_speeds[2] -= 0.01;
            }
            // Right stick X for adjusting front right.
            if (gamepads.getAnalogValues().gamepad2RightStickX > 0.5) {
                navigation.wheel_speeds[3] += 0.01;
            }
            if (gamepads.getAnalogValues().gamepad2RightStickX < -0.5) {
                navigation.wheel_speeds[3] -= 0.01;
            }
        }

        mechanismDriving.adjustDesiredSlideHeight(gamepads.getAnalogValues(), robot);

        robot.telemetry.addData("Front Motor Relative Speeds", "left (%.2f), right (%.2f)",
                navigation.wheel_speeds[2], navigation.wheel_speeds[3]);
        robot.telemetry.addData("Rear Motor Relative Speeds", "left (%.2f), right (%.2f)",
                navigation.wheel_speeds[0], navigation.wheel_speeds[1]);
        robot.telemetry.addData("Movement mode", "" + robot.movementMode.name());

        previousStateGamepads.copyGamepads(gamepads);
    }

    /** Calls all non-blocking FSM methods to read from state and act accordingly.
     */
    public void driveMechanisms() {
       mechanismDriving.updateHorseshoe(robot);
       mechanismDriving.updateSlides(robot, Math.abs(gamepads.getAnalogValues().gamepad2LeftStickY));
       mechanismDriving.updateClaw(robot);
    }

    /** Changes drivetrain motor inputs based off the controller inputs.
     */
    public void maneuver() {
        navigation.updateStrafePower(hasMovementDirection(), gamepads, robot);

        // Only move if one of the D-Pad buttons are pressed or the joystick is not centered.
        boolean movedStraight = navigation.moveStraight(
                gamepads.getButtonState(GamepadWrapper.DriverAction.MOVE_STRAIGHT_FORWARD),
                gamepads.getButtonState(GamepadWrapper.DriverAction.MOVE_STRAIGHT_BACKWARD),
                gamepads.getButtonState(GamepadWrapper.DriverAction.MOVE_STRAIGHT_LEFT),
                gamepads.getButtonState(GamepadWrapper.DriverAction.MOVE_STRAIGHT_RIGHT),
                robot
        );
        if (!movedStraight) {
            navigation.maneuver(
                    gamepads.getAnalogValues(),
                    robot);
        }
    }

    /** Determines whether the button for a particular action was released in the current OpMode iteration.
     */
    private boolean getButtonRelease(GamepadWrapper.DriverAction action) {
        return !gamepads.getButtonState(action) && previousStateGamepads.getButtonState(action);
    }

    // AUTONOMOUS
    // ==========

    /** Moves the robot to the next point of interest.
     */
    public Position travelToNextPOI() {
        return navigation.travelToNextPOI(robot);
    }

    /** Determines the position of the capstone on the barcode.
     */
    public Robot.ParkingPosition readBarcode() {
        // Reset the barcode scanning counters and states
//        robot.barcodeScanResult = Robot.BarcodeScanResult.WRONG_CAPS;
//        robot.resetBarcodeScanMap();
//        robot.numBarcodeAttempts = 0;
//        robot.barcodeScanState = Robot.BarcodeScanState.SCAN;

        // TODO: call CV function

        // Wait for CV to determine a finalized barcodeScanResult value (this is blocking!)
        while (robot.barcodeScanState == Robot.BarcodeScanState.SCAN) {
            try {
                TimeUnit.MICROSECONDS.sleep(10);
            }
            catch (InterruptedException e) {}
        }

        boolean flipped = (allianceColor == AllianceColor.RED && startingSide == StartingSide.OUR_COLOR) || (allianceColor == AllianceColor.BLUE && startingSide == StartingSide.THEIR_COLOR);

        switch (robot.barcodeScanResult) {
            case LEFT: return flipped ? Robot.ParkingPosition.OUTSIDE : Robot.ParkingPosition.INSIDE;
            case RIGHT: return flipped ? Robot.ParkingPosition.INSIDE : Robot.ParkingPosition.OUTSIDE;
            default: return Robot.ParkingPosition.MIDDLE;
        }
    }

    // Each of these methods manually sets the robot state so that a specific task is started, and forces these tasks to
    // be synchronous by repeatedly calling the mechanism driving methods. These are to be used in an autonomous OpMode.

    /** Horse shoe code
     *  This horse shoe code here is part of the linear slides and helps the cone position into the poles.
     */
    public void flipHorseshoe() {
        switch (robot.desiredHorseshoeState) {
            case FRONT: robot.desiredHorseshoeState = Robot.HorseshoeState.REAR;
            case REAR: robot.desiredHorseshoeState = Robot.HorseshoeState.FRONT;
        }
        double startTime = robot.elapsedTime.milliseconds(); //Starts the time of the robot in milliseconds.
        mechanismDriving.updateHorseshoe(robot);
        //case Robot.HorseshoeState.FRONT/REAR (Remove the / in between if needed to be added back. Only set 1 variable at a time)
        //Waiting for servo to finish rotation
        while (robot.elapsedTime.milliseconds() - startTime < MechanismDriving.HORSESHOE_TIME) {}
    }

    /** Opens the claw.
     */
    public void openClaw() {
        robot.desiredClawState = Robot.ClawState.OPEN;
        double startingTime = robot.elapsedTime.milliseconds();
        mechanismDriving.updateClaw(robot);
        // Wait for claw to open.
        while (robot.elapsedTime.milliseconds() - startingTime < MechanismDriving.CLAW_SERVO_TIME) {}
    }

    /** Closes the claw.
     */
    public void closeClaw() {
        robot.desiredClawState = Robot.ClawState.CLOSED;
        double startingTime = robot.elapsedTime.milliseconds();
        mechanismDriving.updateClaw(robot);
        // Wait for claw to close.
        while (robot.elapsedTime.milliseconds() - startingTime < MechanismDriving.CLAW_SERVO_TIME) {}
    }

    /** Finds the lowered SlidesState given a standard SlidesState
     *
     *  @param currentSlidesState the current state of the linear slides
     *  @return the lowered SlidesState (or the retracted SlidesState if it was already retracted)
     */
//    public Robot.SlidesState getLoweredSlidesState(Robot.SlidesState currentSlidesState) {
//        switch (currentSlidesState) {
//            case LOW: return Robot.SlidesState.LOW_LOWERED;
//            case MEDIUM: return Robot.SlidesState.MEDIUM_LOWERED;
//            case HIGH: return Robot.SlidesState.HIGH_LOWERED;
//            default: return Robot.SlidesState.RETRACTED;
//        }
//    }

    /** Delivers a cone to a pole.
     *
     *  @param level the level/height of the pole to which the cone needs to be delivered
     */
    public void deliverToPole(Robot.SlidesState level, Robot robot) {
//        // Extend slides.
//        robot.desiredSlidesState = level;
//        boolean extended = mechanismDriving.updateSlides(robot);
//        while (!extended) {
//            extended = mechanismDriving.updateSlides(robot);
//        }
//        flipHorseshoe();
//
//        // Move into drop-off position.
//        robot.positionManager.updatePosition(robot);
//        Position startPos = new Position(robot.getPosition().getX(), robot.getPosition().getY(),
//                robot.getPosition().getRotation(), "POI startPos");
//
//        //The direction that the robot moves away at will need to depend on which side we are playing on
//        switch(startingSide) {
//            case OUR_COLOR:
//                navigation.path.add(navigation.pathIndex,
//                    new Position(startPos.getX(), startPos.getY() + Navigation.HORSESHOE_SIZE, //horseshoe moves towards junction
//                            startPos.getRotation(), "POI dropoff our color"));
//                break;
//            case THEIR_COLOR:
//                navigation.path.add(navigation.pathIndex,
//                        new Position(startPos.getX(), startPos.getY() - Navigation.HORSESHOE_SIZE,
//                                startPos.getRotation(), "POI dropoff their color"));
//                break;
//        }
//
//        travelToNextPOI();
//
//        //flipHorseshoe(); not needed anymore because no more servo
//
//        // Lower linear slides
////        robot.desiredSlidesState = getLoweredSlidesState(robot.desiredSlidesState);
//        boolean lowered = mechanismDriving.updateSlides(robot);
//        while (!lowered) {
//            lowered = mechanismDriving.updateSlides(robot);
//        }
//
//        // Drop cone
//        openClaw();
//
//        // Move back to starting position.
//        navigation.path.add(navigation.pathIndex, startPos);
//        travelToNextPOI();
//
//        // Retract slides.
//        flipHorseshoe();
//        robot.desiredSlidesState = Robot.SlidesState.RETRACTED;
//        boolean retracted = mechanismDriving.updateSlides(robot);
//        while (!retracted) {
//            retracted = mechanismDriving.updateSlides(robot);
//        }
//
//        // TODO: ROBOT NEEDS TO BACK UP otherwise flipping horeshoe will cause problems
//
//        //flipHorseshoe(); not needed anymore because no more horseshoe
    }

    /** Picks up a cone.
     */
    // TODO: Implement picking up cones from levels higher than the ground level
    public void pickUpCone() {
        closeClaw();
    }

    /** Returns whether the driver is attempting to move the robot linearly
     *
     *  @return boolean whether the d-pad has a button pressed or the joystick is not centered
     */
    public boolean hasMovementDirection() {
        boolean dpadPressed = (gamepads.getButtonState(GamepadWrapper.DriverAction.MOVE_STRAIGHT_FORWARD)
                            || gamepads.getButtonState(GamepadWrapper.DriverAction.MOVE_STRAIGHT_BACKWARD)
                            || gamepads.getButtonState(GamepadWrapper.DriverAction.MOVE_STRAIGHT_LEFT)
                            || gamepads.getButtonState(GamepadWrapper.DriverAction.MOVE_STRAIGHT_RIGHT));
        double stickDist = Math.sqrt(
                Math.pow(gamepads.getAnalogValues().gamepad1LeftStickY,2)
              + Math.pow(gamepads.getAnalogValues().gamepad1LeftStickX,2));
        boolean joystickMoved = stickDist >= RobotManager.JOYSTICK_DEAD_ZONE_SIZE;
        return dpadPressed || joystickMoved;
    }
}

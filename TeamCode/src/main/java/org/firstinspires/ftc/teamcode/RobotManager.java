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
            robot.desiredSlidesState = Robot.SlidesState.RETRACTED;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_LOW)) {
            robot.desiredSlidesState = Robot.SlidesState.LOW;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_MEDIUM)) {
            robot.desiredSlidesState = Robot.SlidesState.MEDIUM;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_HIGH)) {
            robot.desiredSlidesState = Robot.SlidesState.HIGH;
        }

        // Fine movement/rotation.
        if (getButtonRelease(GamepadWrapper.DriverAction.FINE_MOVEMENT_TOGGLE)) {
            if (robot.movementMode == Robot.MovementMode.FINE) {
                robot.movementMode = Robot.MovementMode.NORMAL;
            } else {
                robot.movementMode = Robot.MovementMode.FINE;
            }
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.ULTRA_FINE_MOVEMENT_TOGGLE)) {
            if (robot.movementMode == Robot.MovementMode.ULTRA_FINE) {
                robot.movementMode = Robot.MovementMode.NORMAL;
            } else {
                robot.movementMode = Robot.MovementMode.ULTRA_FINE;
            }
        }

        if (getButtonRelease(GamepadWrapper.DriverAction.TOGGLE_WHEEL_SPEED_ADJUSTMENT)) {
            robot.wheelSpeedAdjustment = !robot.wheelSpeedAdjustment;
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

            // TODO: Add button for compliant wheels (on and off switch)
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
//        mechanismDriving.updateCarousel(robot);
//        mechanismDriving.updateClaw(robot);
//        mechanismDriving.updateSlides(robot);
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
                    gamepads.getButtonState(GamepadWrapper.DriverAction.TURN_COUNTER_CLOCKWISE),
                    gamepads.getButtonState(GamepadWrapper.DriverAction.TURN_CLOCKWISE),
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

    /** Finds the lowered SlidesState given a standard SlidesState
     *
     *  @param currentSlidesState the current state of the linear slides
     *  @return the lowered SlidesState (or the retracted SlidesState if it was already retracted)
     */
    public Robot.SlidesState getLoweredSlidesState(Robot.SlidesState currentSlidesState) {
        switch (currentSlidesState) {
            case LOW: return Robot.SlidesState.LOW_LOWERED;
            case MEDIUM: return Robot.SlidesState.MEDIUM_LOWERED;
            case HIGH: return Robot.SlidesState.HIGH_LOWERED;
            default: return Robot.SlidesState.RETRACTED;
        }
    }

    /** Delivers a piece of freight to a particular level of the alliance shipping hub.
     *
     *  @param level the level to which the cargo needs to be delivered.
     */
    public void deliverToPole(Robot.SlidesState level, Robot robot) {
        // Extend slides.
        robot.desiredSlidesState = level;
        boolean extended = mechanismDriving.updateSlides(robot);
        while (!extended) {
            extended = mechanismDriving.updateSlides(robot);
        }

        // Move into drop-off position.
        robot.positionManager.updatePosition(robot);
        Position startPos = new Position(robot.getPosition().getX(), robot.getPosition().getY(),
                robot.getPosition().getRotation(), "POI startPos");

        navigation.path.add(navigation.pathIndex,
                new Position(startPos.getX() + Navigation.HORSESHOE_SIZE, startPos.getY(),
                        startPos.getRotation(), "POI dropoff"));

        travelToNextPOI();

        flipHorseshoe();

        // Lower linear slides
        robot.desiredSlidesState = getLoweredSlidesState(robot.desiredSlidesState);
        boolean lowered = mechanismDriving.updateSlides(robot);
        while (!lowered) {
            lowered = mechanismDriving.updateSlides(robot);
        }

        // Move back to starting position.
        navigation.path.add(navigation.pathIndex, startPos);
        travelToNextPOI();

        // Retract slides.
        robot.desiredSlidesState = Robot.SlidesState.RETRACTED;
        boolean retracted = mechanismDriving.updateSlides(robot);
        while (!retracted) {
            retracted = mechanismDriving.updateSlides(robot);
        }

        flipHorseshoe();
    }

    /** Picks up a cone.
     */
    public void pickUpCone() {
        // Suck the cone into the horseshoe
        robot.desiredCompliantWheelsState = Robot.CompliantWheelsState.ON;
        double startTime = elapsedTime.milliseconds();
        while (elapsedTime.milliseconds() - startTime < mechanismDriving.COMPLIANT_WHEELS_TIME)
            mechanismDriving.updateCompliantWheels(robot);
        // Turns off compliant wheels
        robot.desiredCompliantWheelsState = Robot.CompliantWheelsState.OFF;
        mechanismDriving.updateCompliantWheels(robot);
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
              + Math.pow(gamepads.getAnalogValues().gamepad1RightStickX,2));
        boolean joystickMoved = stickDist >= RobotManager.JOYSTICK_DEAD_ZONE_SIZE;
        return dpadPressed || joystickMoved;
    }
}

/* Authors: Arin Khare, Kai Vernooy
 */

package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public enum StartingSide {WAREHOUSE, CAROUSEL}

    public Robot robot;

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

        elapsedTime.reset();
        navigation = new Navigation(path, allianceColor, startingSide, movementMode);
        mechanismDriving = new MechanismDriving();

        robot = new Robot(hardwareMap, telemetry, elapsedTime);

        if (!path.isEmpty()) {
            computerVision = new ComputerVision(hardwareMap, new AutonPipeline(robot, telemetry, allianceColor));
        }

        gamepads = new GamepadWrapper(gamepad1, gamepad2);
        previousStateGamepads = new GamepadWrapper();
        previousStateGamepads.copyGamepads(gamepads);

        if (allianceColor == AllianceColor.BLUE) {
            robot.carouselMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    // TELE-OP
    // =======

    /** Determine new robot desired states based on controller input (checks for button releases)
     */
    public void readControllerInputs() {
        // Carousel
        if (getButtonRelease(GamepadWrapper.DriverAction.RUN_CAROUSEL)) {
            switch (robot.desiredCarouselState) {
                case STOPPED:
                    robot.desiredCarouselState = Robot.CarouselState.SPINNING;
                    mechanismDriving.carouselStartTime = elapsedTime.milliseconds();
                    break;
                case SPINNING:
                    robot.desiredCarouselState = Robot.CarouselState.STOPPED;
                    break;
            }
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.TOGGLE_AUTO_SPIN)) {
            if (robot.desiredCarouselState != Robot.CarouselState.AUTO_SPIN) {
                robot.desiredCarouselState = Robot.CarouselState.AUTO_SPIN;
                mechanismDriving.carouselStartTime = elapsedTime.milliseconds();
            }
            else {
                robot.desiredCarouselState = Robot.CarouselState.STOPPED;
            }
        }

        // Claw
        if (getButtonRelease(GamepadWrapper.DriverAction.OPEN_CLAW)) {
            robot.desiredClawState = Robot.ClawState.OPEN;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.CLOSE_CLAW)) {
            robot.desiredClawState = Robot.ClawState.CLOSED;
        }

        // Linear slides
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_RETRACTED)) {
            robot.desiredSlidesState = Robot.SlidesState.RETRACTED;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_L1)) {
            robot.desiredSlidesState = Robot.SlidesState.L1;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_L2)) {
            robot.desiredSlidesState = Robot.SlidesState.L2;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_L3)) {
            robot.desiredSlidesState = Robot.SlidesState.L3;
        }
        if (getButtonRelease(GamepadWrapper.DriverAction.SET_SLIDES_CAPPING)) {
            robot.desiredSlidesState = Robot.SlidesState.CAPPING;
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
        mechanismDriving.updateCarousel(robot);
        mechanismDriving.updateClaw(robot);
        mechanismDriving.updateSlides(robot);
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


    /** Converts a result from the barcode scanner into a level on which to place the preload box
     * @param result The result of the barcode scanning. This will NEVER be WRONG_CAPS or WRONG_TAPE, it will always be a valid barcode state
     * @return A SlidesState that represents the scoring level to deposit to
     */
    private Robot.SlidesState barcodeResultToSlidesState(Robot.BarcodeScanResult result) {

        switch (result) {
            case LEFT: return Robot.SlidesState.L1;
            case CENTER: return Robot.SlidesState.L2;
            case RIGHT: return Robot.SlidesState.L3;
        }

        return Robot.SlidesState.L1;
    }


    /** Determines the position of the capstone on the barcode.
     */
    public Robot.SlidesState readBarcode() {
        // Reset the barcode scanning counters and states
        robot.barcodeScanResult = Robot.BarcodeScanResult.WRONG_CAPS;
        robot.resetBarcodeScanMap();

        robot.barcodeScanState = Robot.BarcodeScanState.SCAN;
        robot.numBarcodeAttempts = 0;

        // Wait for CV to determine a finalized barcodeScanResult value (this is blocking!)
        while (robot.barcodeScanState == Robot.BarcodeScanState.SCAN) {
            try {
                TimeUnit.MICROSECONDS.sleep(10);
            }
            catch (InterruptedException e) {}
        }

        return barcodeResultToSlidesState(robot.barcodeScanResult);
    }


    // Each of these methods manually sets the robot state so that a specific task is started, and forces these tasks to
    // be synchronous by repeatedly calling the mechanism driving methods. These are to be used in an autonomous OpMode.

    /** Delivers a duck by spinning the carousel.
     */
    public void deliverDuck() {
        robot.desiredCarouselState = Robot.CarouselState.STOPPED;
        mechanismDriving.updateCarousel(robot);

        robot.desiredCarouselState = Robot.CarouselState.SPINNING;
        mechanismDriving.updateCarousel(robot);

        int sleepTime = 0;
        for (int interval : MechanismDriving.CAROUSEL_TIMES) {
            sleepTime += interval;
        }

        telemetry.addData("sleepTime", sleepTime);
        telemetry.update();

        double startingTime = robot.elapsedTime.milliseconds();
        while (robot.elapsedTime.milliseconds() - startingTime < sleepTime) {}

        robot.desiredCarouselState = Robot.CarouselState.STOPPED;
        mechanismDriving.updateCarousel(robot);
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

    /** Delivers a piece of freight to a particular level of the alliance shipping hub.
     *
     *  @param level the level to which the cargo needs to be delivered.
     */
    public void deliverToShippingHub(Robot.SlidesState level) {
        // Extend slides.
        robot.desiredSlidesState = level;
        boolean extended = mechanismDriving.updateSlides(robot);
        while (!extended) {
            extended = mechanismDriving.updateSlides(robot);
        }

        // Move into drop-off position.
        robot.positionManager.updatePosition(robot);
        Position startPos = new Position(
                new Point(robot.getPosition().getX(), robot.getPosition().getY(), "POI startPos"),
                robot.getPosition().getRotation());

        double forwardDistance = Navigation.CLAW_SIZE;
        switch (level) {
            case L1:
                forwardDistance -= 3;
                break;
            case L2:
                forwardDistance -= 4;
                break;
            case L3:
                forwardDistance -= 7;
                break;
        }
//        if (level == Robot.SlidesState.L1) {forwardDistance -= 0.75;}

        if (forwardDistance > 0) {
            navigation.path.add(navigation.pathIndex,
                    new Position(new Point(startPos.getX() + forwardDistance, startPos.getY(), "POI dropoff",
                            Point.Action.NONE, 0.4, 0.0), startPos.getRotation()));

            travelToNextPOI();
        }

        openClaw();

        // Move back to starting position.
        navigation.path.add(navigation.pathIndex, startPos);
        travelToNextPOI();

        // Retract slides.
        robot.desiredSlidesState = Robot.SlidesState.RETRACTED;
        boolean retracted = mechanismDriving.updateSlides(robot);
        while (!retracted) {
            retracted = mechanismDriving.updateSlides(robot);
        }

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
              + Math.pow(gamepads.getAnalogValues().gamepad1RightStickX,2));
        boolean joystickMoved = stickDist >= RobotManager.JOYSTICK_DEAD_ZONE_SIZE;
        return dpadPressed || joystickMoved;
    }
}

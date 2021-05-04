package org.firstinspires.ftc.teamcode.driver;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.drivetrain.movement.Movement;
import org.firstinspires.ftc.teamcode.robot.drivetrain.movement.Turn;
import org.firstinspires.ftc.teamcode.robot.drivetrain.wheels.WheelTypes;

/**
 * Drive code for a person controlling the robot, this is a very dumb-downed version.
 */
public class Driver {
    private boolean aPressing = false;

    private boolean useTelemetry;

    public Robot robot;
    private DriveMode driveMode;

    /**
     *
     * @param useTelemetry Pretty straight forward, let this class use telemetry, output stuff?
     * @param driveMode How do you want to drive the car?
     * @param wheelType What type of wheel do you have?
     * @param gamepad1 Gamepad 1
     * @param gamepad2 Gamepad 2
     * @param telemetry Telemetry
     * @param hardwareMap Hardware Map
     */
    public Driver(boolean useTelemetry, DriveMode driveMode, WheelTypes wheelType, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap) {
        this.useTelemetry = useTelemetry;
        robot = new Robot(wheelType, gamepad1, gamepad2,telemetry,hardwareMap);
        robot.setDrivePowerModifier(1);
        this.driveMode = driveMode;
    }

    public Driver(DriveMode driveMode, WheelTypes wheelType, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap) {
        this(true, driveMode, wheelType, gamepad1, gamepad2, telemetry, hardwareMap);
    }

    public Driver(boolean useTelemetry, WheelTypes wheelType, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap) {
        this(useTelemetry, DriveMode.TWO_STICK, wheelType, gamepad1, gamepad2, telemetry, hardwareMap);
    }

    public Driver(WheelTypes wheelType, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap) {
        this(true, wheelType, gamepad1, gamepad2, telemetry, hardwareMap);
    }

    /**
     * Starts the robot. Hook this up to the start method in the OP Mode
     */
    public void start() {
        robot.start();
    }

    /**
     * Stops the robot. Hook this up to the stop method in the OP Mode
     */
    public void stop() {
        robot.stop();
    }

    /**
     * Loop of the robot. Hook this up to the loop method in the OP Mode
     */
    public void loop() {
        // Change Mode
        if (robot.gamepad1.a) { // a pressed
            if (!aPressing) {
                aPressed();
                aPressing = true;
            }
        } else {
            aPressing = false;
        }
        // Say Mode
        if (useTelemetry) robot.telemetry.addData("Driver Mode", driveMode);

        // Move / Drive
        switch (driveMode) {
            case TWO_STICK:
                robot.driveTrain.move(Movement.FORWARDS, -robot.gamepad1.left_stick_y);
                robot.driveTrain.turn(Turn.CLOCKWISE, robot.gamepad1.right_stick_x);
                break;
            case ARCADE:
                robot.driveTrain.move(Movement.FORWARDS, -robot.gamepad1.left_stick_y);
                robot.driveTrain.turn(Turn.CLOCKWISE, robot.gamepad1.left_stick_x);
                break;
            case GTA:
                robot.driveTrain.move(Movement.BACKWARDS, robot.gamepad1.left_trigger); // Backwards
                robot.driveTrain.move(Movement.FORWARDS, robot.gamepad1.right_trigger); // Forwards
                robot.driveTrain.turn(Turn.CLOCKWISE, robot.gamepad1.left_stick_x); // Turning
                break;
            case FORZA:
                double backwardsPower = 1;
                if (robot.gamepad1.y) robot.driveTrain.move(Movement.BACKWARDS, backwardsPower); // Backwards
                robot.driveTrain.move(Movement.FORWARDS, robot.gamepad1.right_trigger); // Forwards
                robot.driveTrain.turn(Turn.CLOCKWISE, robot.gamepad1.left_stick_x); // Turning
                break;
            default:
                stop();
                break;
        }

    }

    private void aPressed() {
        driveMode = driveMode.next();
    }
}

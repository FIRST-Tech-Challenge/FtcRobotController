package org.firstinspires.ftc.teamcode.driver;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.drivetrain.movement.Movement;
import org.firstinspires.ftc.teamcode.robot.drivetrain.movement.Turn;
import org.firstinspires.ftc.teamcode.robot.drivetrain.wheels.WheelTypes;

public class Driver {
    private boolean aPressing = false;

    private boolean useTelemetry;

    public Robot robot;
    private DriveMode driveMode;

    public Driver(boolean useTelemetry, DriveMode driveMode, WheelTypes wheelType, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap) {
        this.useTelemetry = useTelemetry;
        robot = new Robot(wheelType, gamepad1, gamepad2,telemetry,hardwareMap);
        robot.setDrivePowerModifier(1);
        this.driveMode = driveMode;
    }

    public Driver(DriveMode driveMode, WheelTypes wheelType, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap) {
        this(true, driveMode, wheelType, gamepad1, gamepad2, telemetry, hardwareMap);
    }


    public void start() {
        robot.start();
    }

    public void stop() {
        robot.stop();
    }

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
        }

    }

    private void aPressed() {
        driveMode = driveMode.next();
    }
}

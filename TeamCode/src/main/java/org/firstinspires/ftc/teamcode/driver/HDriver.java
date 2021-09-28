package org.firstinspires.ftc.teamcode.driver;

import org.firstinspires.ftc.teamcode.buttons.ButtonMGR;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.drivetrains.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.Movement;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.Turn;

/**
 * Drive code for a person controlling the robot with HDrive
 * @author 22jmiller
 */
public class HDriver extends Driver {
    private boolean useTelemetry;

    private HDriveMode hDriveMode;

    private ButtonMGR buttonMGR;

    public HDriver(boolean useTelemetry, HDriveMode hDriveMode, Robot robot) {
        super(robot);
        this.useTelemetry = useTelemetry;
        this.hDriveMode = hDriveMode;
    }

    @Override
    public void loop() {
        buttonMGR = new ButtonMGR(robot.telemetry, robot);
        robot.telemetry.addData("Mode", hDriveMode);
        // Move / Drive
        switch (hDriveMode) {
            case MINECRAFT:
                // Move
                ((DriveTrain)robot.getRobotPart(DriveTrain.class)).move(Movement.FORWARDS, -robot.gamepad1.left_stick_y);
                ((DriveTrain)robot.getRobotPart(DriveTrain.class)).move(Movement.RIGHT, robot.gamepad1.left_stick_x);

                // Turn
                ((DriveTrain)robot.getRobotPart(DriveTrain.class)).turn(Turn.CLOCKWISE, robot.gamepad1.right_stick_x);

                buttonMGR.CheckAndExecute();

                break;
            default:
                stop();
                break;
        }
    }
}

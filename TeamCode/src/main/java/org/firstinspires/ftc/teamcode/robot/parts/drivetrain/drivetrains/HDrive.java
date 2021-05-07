package org.firstinspires.ftc.teamcode.robot.parts.drivetrain.drivetrains;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.Movement;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.MovementException;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.MovementExceptionReason;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.Turn;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.wheels.WheelException;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.wheels.WheelTypes;

/**
 * Robot Part that will control H Drive trains
 * Accepted Wheel types | Omni
 * @author 22jmiller
 */
public class HDrive extends DriveTrain{
    public HDrive(boolean useTelemetry, WheelTypes wheelType, Robot robot) {
        super(useTelemetry, wheelType, robot);
    }

    @Override
    public void moveMecanum(Movement movementType, double power) {
        throw new WheelException();
    }

    @Override
    public void moveRubber(Movement movementType, double power) {
        switch (movementType) {
            case FORWARDS:
            case BACKWARDS:
                moveOmni(movementType, power);
            default:
                throw new MovementException(MovementExceptionReason.NOT_ALLOWED_MOVE);
        }
    }

    @Override
    public void moveOmni(Movement movementType, double power) {
        switch (movementType) {
            case BACKWARDS:
                power *= -1;
            case FORWARDS:
                // Move
                setLeftRightMotorsPower(power);
                break;
            case LEFT:
                power *= -1;
            case RIGHT:
                setCenterMotorsPower(power);
                break;
            default:
                throw new MovementException(MovementExceptionReason.NOT_ALLOWED_MOVE);
        }
    }

    @Override
    public void turnMecanum(Turn turnType, double power) {
        throw new WheelException();
    }

    @Override
    public void turnRubber(Turn turnType, double power) {
        throw new WheelException();
    }

    @Override
    public void turnOmni(Turn turnType, double power) {
        switch (turnType) {
            case COUNTERCLOCKWISE:
                power *= -1;
            case CLOCKWISE:
                setLeftRightMotorsPower(power,-power);
                break;
            default:
                // Not allowed Move
                MovementException exception = new MovementException(MovementExceptionReason.NOT_ALLOWED_MOVE);
                throw(exception);
        }
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }
}

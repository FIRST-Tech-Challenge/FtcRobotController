package org.firstinspires.ftc.teamcode.robot.parts.drivetrain.drivetrains;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.Movement;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.MovementException;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.MovementExceptionReason;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.Turn;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.wheels.WheelTypes;

/**
 * Robot Part that will control H Drive trains
 * Accepted Wheel types | Rubber Mecanum Omni
 * @author 22jmiller
 */
public class StandardDrive extends DriveTrain {

    private static final WheelTypes acceptedWheels[] = {WheelTypes.RUBBER,WheelTypes.MECANUM,WheelTypes.OMNI};

    public StandardDrive(boolean useTelemetry, WheelTypes wheelType, Robot robot) {
        super(useTelemetry, wheelType, robot);
    }

    // TODO: Merge with current
    public void moveMecanum(Movement movementType, double power) {

    }

    public void moveRubber(Movement movementType, double power) {
        moveRubberOmni(movementType, power);
    }

    public void moveOmni(Movement movementType, double power) {
        moveRubberOmni(movementType,power);
    }

    @Override
    public void turnMecanum(Turn turnType, double power) {

    }

    @Override
    public void turnRubber(Turn turnType, double power) {
        turnRubberOmni(turnType,power);
    }

    @Override
    public void turnOmni(Turn turnType, double power) {
        turnRubberOmni(turnType,power);
    }

    public void moveRubberOmni(Movement movementType, double power) {
        Movement[] movementsAllowed = {Movement.FORWARDS,Movement.BACKWARDS};

        switch (movementType) {
            case BACKWARDS:
                power *= -1;
            case FORWARDS:
                // Move
                setLeftRightMotorsPower(power);
                break;
            default:
                // Not allowed Move
                MovementException exception = new MovementException(MovementExceptionReason.NOT_ALLOWED_MOVE);
                throw(exception);
        }
    }

    public void turnRubberOmni(Turn turnType, double power) {
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

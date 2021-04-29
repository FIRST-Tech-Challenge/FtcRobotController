package org.firstinspires.ftc.teamcode.robot.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.drivetrain.movement.Movement;
import org.firstinspires.ftc.teamcode.robot.drivetrain.movement.MovementException;
import org.firstinspires.ftc.teamcode.robot.drivetrain.movement.MovementExceptionReason;
import org.firstinspires.ftc.teamcode.robot.drivetrain.movement.Turn;
import org.firstinspires.ftc.teamcode.robot.drivetrain.wheels.WheelTypes;
import org.firstinspires.ftc.teamcode.settings.DeviceNames;

import java.util.ArrayList;

public class DriveTrain {
    // Telemetry
    private boolean useTelemetry;

    // Motors
    DcMotor[] leftMotors;
    DcMotor[] rightMotors;

    // Wheels
    WheelTypes wheelType;

    // Power Modifier
    double powerModifier;

    public DriveTrain(boolean useTelemetry, WheelTypes wheelType, DcMotor[] leftMotors, DcMotor[] rightMotors) {
        this.useTelemetry = useTelemetry;
        this.wheelType = wheelType;
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
        this.powerModifier = 1;
    }

    public DriveTrain(boolean useTelemetry, WheelTypes wheelType, HardwareMap hardwareMap, String[] leftMotorNames, String[] rightMotorNames) {
        this.useTelemetry = useTelemetry;
        this.wheelType = wheelType;
        this.powerModifier = 1;

        // Left Drive
        ArrayList<DcMotor> leftMotorsArrayList = new ArrayList<DcMotor>();
        for (String name : DeviceNames.LEFT_DRIVE) {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            // TODO See if this is the one to be in reverse or the other side
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftMotorsArrayList.add(motor);
        }
        leftMotors = leftMotorsArrayList.toArray(new DcMotor[leftMotorsArrayList.size()]);

        // Right Drive
        ArrayList<DcMotor> rightMotorsArrayList = new ArrayList<DcMotor>();
        for (String name : DeviceNames.RIGHT_DRIVE) {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            rightMotorsArrayList.add(motor);
        }
        rightMotors = rightMotorsArrayList.toArray(new DcMotor[rightMotorsArrayList.size()]);
    }

    public DriveTrain(boolean useTelemetry, WheelTypes wheelType, Robot robot) {
        this(useTelemetry, wheelType, robot.hardwareMap, robot.deviceNames.LEFT_DRIVE, robot.deviceNames.RIGHT_DRIVE);
    }

    public void setUseTelemetry(boolean useTelemetry) {
        this.useTelemetry = useTelemetry;
    }

    public void setPowerModifier(double powerModifier) {
        this.powerModifier = powerModifier;
    }

    public void stop() {
        for (DcMotor leftMotor : leftMotors) {
            leftMotor.setPower(0);
        }

        for (DcMotor rightMotor : rightMotors) {
            rightMotor.setPower(0);
        }
    }

    public void move(Movement movementType, double power) {
        power *= powerModifier;
        switch (wheelType) {
            case MECANUM:
                moveMecanum(movementType,power);
                break;
            case RUBBER:
                moveRubber(movementType,power);
                break;
        }
    }

    private void moveMecanum(Movement movementType, double power) {

    }

    private void moveRubber(Movement movementType, double power) {
        Movement[] movementsAllowed = {Movement.FORWARDS,Movement.BACKWARDS};

        switch (movementType) {
            case BACKWARDS:
                power *= -1;
            case FORWARDS:
                // Move
                for (DcMotor leftMotor : leftMotors) {
                    leftMotor.setPower(power);
                }

                for (DcMotor rightMotor : rightMotors) {
                    rightMotor.setPower(power);
                }
                break;
            default:
                // Not allowed Move
                MovementException exception = new MovementException(MovementExceptionReason.NOT_ALLOWED_MOVE);
                throw(exception);
        }
    }

    public void turn(Turn turnType, double power) {
        power *= powerModifier;
        switch (wheelType) {
            case MECANUM:
                turnMecanum(turnType,power);
                break;
            case RUBBER:
                turnRubber(turnType,power);
                break;
        }
    }

    private void turnMecanum(Turn turnType, double power) {

    }

    private void turnRubber(Turn turnType, double power) {
        switch (turnType) {
            case COUNTERCLOCKWISE:
                power *= -1;
            case CLOCKWISE:
                for (DcMotor leftMotor : leftMotors) {
                    leftMotor.setPower(-power);
                }

                for (DcMotor rightMotor : rightMotors) {
                    rightMotor.setPower(power);
                }
                break;
        }
    }

}
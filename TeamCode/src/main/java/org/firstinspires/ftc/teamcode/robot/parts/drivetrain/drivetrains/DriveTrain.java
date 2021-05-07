package org.firstinspires.ftc.teamcode.robot.parts.drivetrain.drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.parts.RobotPart;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.Movement;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.Turn;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.wheels.WheelException;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.wheels.WheelTypes;

import java.util.ArrayList;

/**
 * Robot Part that will control Drive trains in general
 * @author 22jmiller
 */
public abstract class DriveTrain extends RobotPart {
    // Telemetry
    private boolean useTelemetry;
    private Telemetry telemetry;

    // Physical items
    HardwareMap hardwareMap;
    DcMotor leftMotors[];
    DcMotor rightMotors[];
    DcMotor centerMotors[];

    // Wheels
    WheelTypes wheelType;

    // Power Modifier
    double powerModifier = 1;

    public DriveTrain(boolean useTelemetry, WheelTypes wheelType, Robot robot) {
        this(useTelemetry,wheelType,robot,true,true);
    }

    public DriveTrain(boolean useTelemetry, WheelTypes wheelType, Robot robot, boolean leftForwards, boolean centerForwards) {
        this.hardwareMap = robot.hardwareMap;
        this.useTelemetry = useTelemetry;
        this.telemetry = robot.telemetry;
        this.wheelType = wheelType;

        if (leftForwards) {
            leftMotors = getMotorArray(robot.deviceNames.LEFT_DRIVE, DcMotorSimple.Direction.FORWARD);
            rightMotors = getMotorArray(robot.deviceNames.RIGHT_DRIVE, DcMotorSimple.Direction.REVERSE);
        } else {
            leftMotors = getMotorArray(robot.deviceNames.LEFT_DRIVE, DcMotorSimple.Direction.REVERSE);
            rightMotors = getMotorArray(robot.deviceNames.RIGHT_DRIVE, DcMotorSimple.Direction.FORWARD);
        }

        if (centerForwards) {
            centerMotors = getMotorArray(robot.deviceNames.CENTER_DRIVE, DcMotorSimple.Direction.FORWARD);
        } else {
            centerMotors = getMotorArray(robot.deviceNames.CENTER_DRIVE, DcMotorSimple.Direction.REVERSE);
        }

    }

    /**
     * Converts names of motors into a DcMotor array
     * @param deviceNames Names of the Motors to add
     * @param direction Direction the motor will go
     * @see com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
     * @return
     */
    private DcMotor[] getMotorArray(String[] deviceNames, DcMotorSimple.Direction direction) {
        ArrayList<DcMotor> motorsArrayList = new ArrayList<DcMotor>();
        for (String name : deviceNames) {
            try { // to add it, if it isn't there don't add it
                DcMotor motor = hardwareMap.get(DcMotor.class, name);
                motor.setDirection(direction);
                motorsArrayList.add(motor);
            } catch (IllegalArgumentException e) {}
        }
        return motorsArrayList.toArray(new DcMotor[motorsArrayList.size()]);
    }

    public void setUseTelemetry(boolean useTelemetry) {
        this.useTelemetry = useTelemetry;
    }

    public void setPowerModifier(double powerModifier) {
        this.powerModifier = powerModifier;
    }

    public void stop() {
        setAllMotorsPower(0);
    }

    public void move(Movement movementType, double power) {
        switch (wheelType) {
            case MECANUM:
                moveMecanum(movementType,power);
                break;
            case RUBBER:
                moveRubber(movementType,power);
                break;
            case OMNI:
                moveOmni(movementType,power);
                break;
            default:
                throw new WheelException("Wheel not found");
        }
        updateTelemetry();
    }

    public abstract void moveMecanum(Movement movementType, double power);

    public abstract void moveRubber(Movement movementType, double power);

    public abstract void moveOmni(Movement movementType, double power);

    public void turn(Turn turnType, double power) {
        switch (wheelType) {
            case MECANUM:
                turnMecanum(turnType,power);
                break;
            case RUBBER:
                turnRubber(turnType,power);
                break;
            case OMNI:
                turnOmni(turnType,power);
                break;
            default:
                throw new WheelException("Wheel not found");
        }
        updateTelemetry();
    }

    public abstract void turnMecanum(Turn turnType, double power);
    public abstract void turnRubber(Turn turnType, double power);
    public abstract void turnOmni(Turn turnType, double power);

    public void setAllMotorsPower(double power) {
        setLeftMotorsPower(power);
        setCenterMotorsPower(power);
        setRightMotorsPower(power);
    }

    public void setLeftRightMotorsPower(double power) {
        setLeftRightMotorsPower(power, power);
    }

    public void setLeftRightMotorsPower(double leftPower, double rightPower) {
        setLeftMotorsPower(leftPower);
        setRightMotorsPower(rightPower);
    }

    public void setLeftMotorsPower(double power) {
        power *= powerModifier;
        for (DcMotor leftMotor : leftMotors) {
            leftMotor.setPower(power);
        }
    }

    public void setRightMotorsPower(double power) {
        power *= powerModifier;
        for (DcMotor rightMotor : rightMotors) {
            rightMotor.setPower(power);
        }
    }
    public void setCenterMotorsPower(double power) {
        power *= powerModifier;
        for(DcMotor centerMotor : centerMotors) {
            centerMotor.setPower(power);
        }
    }

    public boolean hasLeftMotor() {
        return leftMotors.length > 0;
    }

    public boolean hasRightMotor() {
        return rightMotors.length > 0;
    }

    public boolean hasCenterMotors() {
        return centerMotors.length > 0;
    }

    private void updateTelemetry() {
        if (useTelemetry) {
            String lMotorOut = "";
            if (leftMotors.length != 0) {
                double leftMotorPowerPercent = ((int)(leftMotors[0].getPower()*10000))/100;
                lMotorOut = leftMotorPowerPercent + "% L";
            }

            String rMotorOut = "";
            if (rightMotors.length != 0) {
                double rightMotorPowerPercent = ((int)(rightMotors[0].getPower()*10000))/100;
                rMotorOut = rightMotorPowerPercent + "% R";
            }

            String cMotorOut = "";
            if (centerMotors.length != 0) {
                double centerMotorPowerPercent = ((int)(centerMotors[0].getPower()*10000))/100;
                cMotorOut = centerMotorPowerPercent + "% C";
            }

            telemetry.addData("Motors", lMotorOut + " | " + rMotorOut + " | " + cMotorOut + "%");
        }
    }

}
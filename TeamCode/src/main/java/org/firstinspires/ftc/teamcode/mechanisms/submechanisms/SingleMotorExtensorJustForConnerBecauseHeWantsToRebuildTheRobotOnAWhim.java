package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

public class SingleMotorExtensorJustForConnerBecauseHeWantsToRebuildTheRobotOnAWhim implements Extensor {
    private final DcMotor singleMotorExtensorJustForConnerBecauseHeRebuiltTheRobotOnAWhimMotor;
    private final BaseRobot baseRobot;
    private Position currentPosition;
    private final String LOG_PREFIX = "Extensor: ";

    public SingleMotorExtensorJustForConnerBecauseHeWantsToRebuildTheRobotOnAWhim(@NonNull BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.singleMotorExtensorJustForConnerBecauseHeRebuiltTheRobotOnAWhimMotor = baseRobot.hardwareMap
                .get(DcMotor.class, Settings.Hardware.IDs.EXTENSOR_LEFT);

        // Reset encoders
        singleMotorExtensorJustForConnerBecauseHeRebuiltTheRobotOnAWhimMotor
                .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setPosition(Position.HOVER);

        // Set to RUN_TO_POSITION mode for position control
        singleMotorExtensorJustForConnerBecauseHeRebuiltTheRobotOnAWhimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        singleMotorExtensorJustForConnerBecauseHeRebuiltTheRobotOnAWhimMotor
                .setDirection(DcMotorSimple.Direction.REVERSE);
        setPosition(Position.HOVER);
    }

    // Sets target position
    public void setPosition(double position) {
        int targetPosition = (int) position;
        int singleMotorExtensorJustForConnerBecauseHeRebuiltTheRobotOnAWhimMotorCurrent = singleMotorExtensorJustForConnerBecauseHeRebuiltTheRobotOnAWhimMotor
                .getCurrentPosition();

        singleMotorExtensorJustForConnerBecauseHeRebuiltTheRobotOnAWhimMotor.setTargetPosition(targetPosition);
        singleMotorExtensorJustForConnerBecauseHeRebuiltTheRobotOnAWhimMotor
                .setPower(Settings.Hardware.Extensor.MOVEMENT_POWER);

        baseRobot.logger.update(LOG_PREFIX + "Target", String.valueOf(targetPosition));
        baseRobot.logger.update(LOG_PREFIX + "Motor Current",
                String.valueOf(singleMotorExtensorJustForConnerBecauseHeRebuiltTheRobotOnAWhimMotorCurrent));
        baseRobot.logger.update(LOG_PREFIX + "Power", String.valueOf(Settings.Hardware.Extensor.MOVEMENT_POWER));
    }

    // Converts position name to double
    @Override
    public void setPosition(@NonNull Position position) {
        this.currentPosition = position;
        this.setPosition(position.getValue()); // Use the value associated with the enum
    }

    public void extend() {
        // Move to the next position in the enum, looping back to the start if needed
        Position[] positions = Position.values();
        int nextIndex = (currentPosition.ordinal() + 1) % positions.length;
        setPosition(positions[nextIndex]);
    }

    @Override
    public void retract() {
        // Move to the previous position in the enum, looping back if needed
        Position[] positions = Position.values();
        int prevIndex = (currentPosition.ordinal() - 1 + positions.length) % positions.length;
        setPosition(positions[prevIndex]);
    }

    @Override
    public void ground() {
        setPosition(Position.PICKUP);
    }

    @Override
    public void ceiling() {
        setPosition(Position.HIGH_RUNG);
    }
}

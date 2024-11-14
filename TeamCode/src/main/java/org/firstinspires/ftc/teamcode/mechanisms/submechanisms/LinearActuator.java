package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

public class LinearActuator  {
    private final DcMotor actuatorMotor;
    private final BaseRobot baseRobot;
    public LinearActuator(@NonNull BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.actuatorMotor = baseRobot.hardwareMap.get(DcMotor.class, Settings.Hardware.IDs.ACTUATOR);

        // Reset encoders
        actuatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        retract();

        // Set to RUN_TO_POSITION mode for position control
        actuatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void extend() {
        actuatorMotor.setTargetPosition(Settings.Hardware.LinearActuator.MAX);
    }

    public void retract() {
        actuatorMotor.setTargetPosition(Settings.Hardware.LinearActuator.MIN);
    }

    public void stop() {
        actuatorMotor.setTargetPosition(actuatorMotor.getCurrentPosition());
    }
}

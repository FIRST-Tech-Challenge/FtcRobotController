package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config;

public class ElevatorSystem extends SubsystemBase {
    Motor motor;
    ElevatorFeedforward feedforward;

    double target = 0;

    public ElevatorSystem(final HardwareMap hardwareMap) {
        motor = new Motor(hardwareMap, "elevator");
    }

    @Override
    public void periodic() {

    }

    public void setTarget(double input) {
        target = input;
    }
}

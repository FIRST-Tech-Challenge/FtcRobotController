package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config;

public class ElevatorSystem extends SubsystemBase {
    Motor motor;
    ElevatorFeedforward feedforward;
    
    private double velocity = 0;

    public ElevatorSystem(final HardwareMap hardwareMap) {
        motor = new Motor(hardwareMap, Config.MOTOR_ELEVATOR);
        
        feedforward = new ElevatorFeedforward(
            Config.ELEVATOR_KS,
            Config.ELEVATOR_KG,
            Config.ELEVATOR_KV
        );
    }

    @Override
    public void periodic() {
        motor.set(feedforward.calculate(velocity));
    }

    public void setVelocity(double input) {
        velocity = input;
    }
}

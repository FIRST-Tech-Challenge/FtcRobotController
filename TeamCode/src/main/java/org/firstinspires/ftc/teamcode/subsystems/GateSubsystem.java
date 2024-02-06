package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;

public class GateSubsystem extends SubsystemBase {

    Servo leftServo, rightServo;

    public GateSubsystem(final HashMap<RobotHardwareInitializer.Gate, Servo> gate) {
        this.leftServo = gate.get(RobotHardwareInitializer.Gate.LEFT);
        this.rightServo = gate.get(RobotHardwareInitializer.Gate.RIGHT);
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);
    }

    public enum GateState {
        OPEN(.1),
        CLOSED(.9);

        public final double position;
        GateState(double position) {
            this.position = position;
        }

    }

    public void setGateState(GateState state) {
        // TODO: Make the gate command
        leftServo.setPosition(state.position);
        rightServo.setPosition(state.position);
    }

}

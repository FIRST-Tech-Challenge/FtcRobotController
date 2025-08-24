package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {

    public enum ArmState {
        HOME(0),
        HOLD(0.3),
        SCORE(0.4);

        public double pos;

        private ArmState(double pos) {
            this.pos = pos;
        }

    }

    private Servo servo;

    public Arm(Servo servo) {
        this.servo = servo;
    }

    public Command goToPos(ArmState state) {
        return new InstantCommand(() -> this.servo.setPosition(state.pos));
    }












}

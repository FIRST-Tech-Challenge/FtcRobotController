package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeExt extends SubsystemBase {

    public enum IntakeState{
        HOME(0),
        EXTENDED(1);

        public double pos;

        private IntakeState(double pos){this.pos = pos;}
    }
    private Servo intakeExtRight;
    private Servo intakeExtLeft;

    public IntakeExt(HardwareMap hMap) {
        this.intakeExtRight = hMap.get(Servo.class, "intakeExtRight");
        this.intakeExtLeft =  hMap.get(Servo.class, "intakeExtLeft");


    }


    public Command extendIntake() {

        return new InstantCommand(() -> {
            intakeExtLeft.setPosition(IntakeState.EXTENDED.pos);
            intakeExtRight.setPosition(IntakeState.EXTENDED.pos);

        });
    }


    public Command retractIntake(){
        return new InstantCommand(()-> {
            intakeExtRight.setPosition(IntakeState.HOME.pos);
            intakeExtLeft.setPosition(IntakeState.HOME.pos);

        });

    }







}

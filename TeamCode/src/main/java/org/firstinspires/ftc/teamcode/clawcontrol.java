package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class clawcontrol {
    private Servo clamp;
    private Servo LeftDiffyServo;
    private Servo RightDiffyServo;
    public clawcontrol(HardwareMap hardwareMap){
        clamp = hardwareMap.get(Servo.class, "Clamp");
        LeftDiffyServo = hardwareMap.get(Servo.class, "LeftDiffyServo");
        RightDiffyServo = hardwareMap.get(Servo.class, "RightDiffyServo");
    }
    public void closeClaw(){
        clamp.setPosition(0.5);
    }
    public void openClaw(){
        clamp.setPosition((double) 25 /355);
    }

    public void clawDown(){
        wristMotion(0.5, 0.5);
    }
    public void clawPosScoreHighBasket(double leftDiffy, double rightDiffy){
        wristMotion(leftDiffy, rightDiffy);
    }
    public void clawPosScoreChambers(double leftDiffy, double rightDiffy){
        wristMotion(leftDiffy, rightDiffy);
    }
    private void wristMotion(double LeftDiffyPos, double RightDiffyPos){
        //code for the wrist motion move both the servos in parallel
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new InstantCommand(() -> RightDiffyServo.setPosition(RightDiffyPos)),
                        new InstantCommand(() -> LeftDiffyServo.setPosition(LeftDiffyPos))
                )
        );
    }
}

package org.firstinspires.ftc.teamcode.drive.modules;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private final Servo lArm, rArm;

    // need to change later, riptide util?
    private double armAng = 0;

    public Arm(Servo lClawArm, Servo rClawArm){
        this.lArm = lClawArm;
        this.rArm = rClawArm;
    }

    // getter setters of angle and position idk what else to do
}

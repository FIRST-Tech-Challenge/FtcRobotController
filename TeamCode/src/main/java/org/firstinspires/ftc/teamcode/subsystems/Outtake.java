package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class Outtake implements Subsystem{
    //Constants
    double syncFactor=1.05;
    double resetPositionL = 0.5;
    double resetPositionR = 0.5186; // + means up and towards intake
    //BELOW NOT TESTED YET
    double carryL = 0.4057; double carryR=0.61835;
    double dumpL = 0.3756; double dumpR = 0.64922;

    //Hardware
    private Servo armServoL;
    private Servo armServoR;  //    '/[L^R]\'


    public Outtake(Robot robot) {
        armServoL = robot.getServo("armServoL");
        armServoR = robot.getServo("armServoR");

    }

    //TODO: convert left/right positions?



    //TODO: for Servo Sync only
    public void moveR(double d){
        //armServoL.setPosition(0.5);
        armServoR.setPosition(armServoR.getPosition()+(0.001*d));
    }
    public void moveArm(double d){
        armServoL.setPosition(armServoL.getPosition()+(0.001*-d)); //2 degrees??
        armServoR.setPosition(armServoR.getPosition()+(0.001*d*syncFactor));
    }

    public void resetPos(){
        armServoL.setPosition(resetPositionL);
        armServoR.setPosition(resetPositionR);
    }
    public double getRightServoPos() {
        return armServoR.getPosition();
    }
    public double getLeftServoPos(){
        return armServoL.getPosition();
    }

    @Override
    public void update(TelemetryPacket packet) {}
}

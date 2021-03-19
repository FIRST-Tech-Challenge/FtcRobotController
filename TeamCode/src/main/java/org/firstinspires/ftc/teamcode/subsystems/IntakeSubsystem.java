package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Subsystem;
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;

public class IntakeSubsystem extends Subsystem {

    private UpliftRobot robot;
    private DcMotor intake;
    private Servo intakeLifter;


    public IntakeSubsystem(UpliftRobot robot){
        super(robot);
        this.robot = robot;
        this.intake = robot.intake;
        this.intakeLifter = robot.intakeLifter;
    }

    @Override
    public void enable() {

    }

    @Override
    public void disable() {
        setIntakePower(0);
    }

    @Override
    public void stop() {

    }

    @Override
    public void safeDisable() {
        setIntakePower(0);
    }

    public void setIntakePower(double power){
        intake.setPower(power);
    }
    
//    public void IntakeCount(){
//        if (robot.intakeSensor.getDistance(DistanceUnit.CM)<5.48){
//            while(robot.intakeSensor.getDistance(DistanceUnit.CM)<5.48){
//                Utils.sleep(10);
//            }
//            robot.count += 1;
//        }
//    }

    public void liftRoller() {
        intakeLifter.setPosition(0.9);
    }

    public void dropRoller() {
        intakeLifter.setPosition(0.7);
    }
}

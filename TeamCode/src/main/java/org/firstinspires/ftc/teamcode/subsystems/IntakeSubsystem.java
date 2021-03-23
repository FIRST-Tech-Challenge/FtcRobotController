package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Subsystem;

public class IntakeSubsystem extends Subsystem {

    private UpliftRobot robot;
    private DcMotor intake;
    private Servo intakeLifter;
    public Servo sweeperJoint;
    public Servo stick;
    public CRServo sweeperLeft;


    public IntakeSubsystem(UpliftRobot robot){
        super(robot);
        this.robot = robot;
        this.intake = robot.intake;
        this.intakeLifter = robot.intakeLifter;
        this.sweeperJoint = robot.sweeperJoint;
        this.stick = robot.stick;
        this.sweeperLeft = robot.sweeperLeft;
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

    public double getPower(){
        return -intake.getPower();
    }
    
//    public void IntakeCount(){
//        if (robot.intakeSensor.getDistance(DistanceUnit.CM)<5.48){
//            while(robot.intakeSensor.getDistance(DistanceUnit.CM)<5.48){
//                Utils.sleep(10);
//            }
//            robot.count += 1;
//        }
//    }

    public void initRoller() {
        intakeLifter.setPosition(1);
    }

    public void liftRoller() {
        intakeLifter.setPosition(0.85);
    }

    public void dropRoller() {
        intakeLifter.setPosition(0.65);
    }

    public void initStick() {
        stick.setPosition(0.1);
    }

    public void dropStick() {
        stick.setPosition(1);
    }

    public void raiseStick() {
        stick.setPosition(0.9);
    }

    public void initSweeper() {
        sweeperJoint.setPosition(0.5);
    }

    public void dropSweeper() {
        sweeperJoint.setPosition(0.175);
    }

    public void sweeperOn() {
        sweeperLeft.setPower(-1);
    }

    public void sweeperOff() {
        sweeperLeft.setPower(0);
    }
}

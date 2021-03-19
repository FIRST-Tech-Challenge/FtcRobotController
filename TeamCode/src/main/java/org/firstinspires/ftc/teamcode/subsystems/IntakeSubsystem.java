package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
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
    public Servo stickLeft;
    public Servo stickRight;
    public CRServo sweeperLeft;
    public CRServo sweeperRight;


    public IntakeSubsystem(UpliftRobot robot){
        super(robot);
        this.robot = robot;
        this.intake = robot.intake;
        this.intakeLifter = robot.intakeLifter;
        this.stickLeft = robot.stickLeft;
        this.stickRight = robot.stickRight;
        this.sweeperLeft = robot.sweeperLeft;
        this.sweeperRight = robot.sweeperRight;
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

    public void initRoller() {
        intakeLifter.setPosition(1);
    }

    public void liftRoller() {
        intakeLifter.setPosition(0.85);
    }

    public void dropRoller() {
        intakeLifter.setPosition(0.65);
    }

    public void dropSticks() {
        stickLeft.setPosition(0.175);
        stickRight.setPosition(0.825);
    }

    public void raiseSticks() {
        stickLeft.setPosition(0.5);
        stickRight.setPosition(0.5);
    }

    public void sweepersOn() {
        sweeperLeft.setPower(1);
        sweeperRight.setPower(-1);
    }
}

package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.UpliftRobot;

public class IntakeFunctions {

    UpliftRobot robot;
    DcMotor intake;
    Servo intakeLifter;
    Servo sweeperJoint;
    Servo stick;
    CRServo sweeperLeft;
    CRServo sweeperRight;

    public IntakeFunctions(UpliftRobot robot){
        this.robot = robot;
        this.intake = robot.intake;
        this.intakeLifter = robot.intakeLifter;
        this.sweeperJoint = robot.sweeperJoint;
        this.stick = robot.stick;
        this.sweeperLeft = robot.sweeperLeft;
        this.sweeperRight = robot.sweeperRight;
    }

    public void setIntakePower(double power){
        intake.setPower(power);
    }

    public void initRoller() {
        intakeLifter.setPosition(0.95);
    }

    public void liftRoller() {
        intakeLifter.setPosition(0.82);
    }

    public void dropRoller() {
        intakeLifter.setPosition(0.64);
    }

    public void initStick() {
        stick.setPosition(0.035);
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
        sweeperJoint.setPosition(0.17);
    }

    public void sweeperOn() {
        sweeperLeft.setPower(-1);
        sweeperRight.setPower(-1);
    }

    public void sweeperOff() {
        sweeperLeft.setPower(0);
        sweeperRight.setPower(0);
    }

}

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;
@TeleOp (name = "NitinOp", group = "Opmodes")
public class NitinTest extends UpliftTele {
    UpliftRobot robot;

    DcMotor rf, lf, rb, lb, tm;
    Servo intakeLifter, wobbleLeft, wobbleRight, flicker, clamp, sweeperJoint, stick;
    DcMotor intake;
    DcMotorEx transfer;
    DcMotorEx shooter1, shooter2;
    AnalogInput potentiometer;
    DigitalChannel touchBottom;
    DigitalChannel touchTop;
    CRServo sweeperLeft, sweeperRight;
    DistanceSensor shooterSensor;


    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        lf = robot.leftFront;
        rf = robot.rightFront;
        lb = robot.leftBack;
        rb = robot.rightBack;
        tm = robot.testMotor;
        shooter1 = robot.shooter1;
        intake = robot.intake;
        transfer = robot.transfer;
        potentiometer = robot.potentiometer;
        touchBottom = robot.digitalTouchBottom;
        touchTop = robot.digitalTouchTop;
        sweeperLeft = robot.sweeperLeft;
        sweeperRight = robot.sweeperRight;
        shooterSensor = robot.shooterSensor;
        intakeLifter = robot.intakeLifter;
        wobbleLeft = robot.wobbleLeft;
        wobbleRight = robot.wobbleRight;
        flicker = robot.flicker;
        clamp = robot.clamp;
        sweeperJoint = robot.sweeperJoint;
        stick = robot.stick;
        shooter2 = robot.shooter2;

    }

    @Override
    public void initAction() {

    }

    @Override
    public void bodyLoop() {
        tm.setPower(gamepad1.right_stick_y);

    }

    @Override
    public void exit() {

    }
}

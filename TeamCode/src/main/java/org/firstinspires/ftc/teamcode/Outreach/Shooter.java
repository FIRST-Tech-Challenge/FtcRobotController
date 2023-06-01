package org.firstinspires.ftc.teamcode.Outreach;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.awt.font.NumericShaper;

public class Shooter
{
    MechanicalDriveOutreach mechanicalDriveOutreach;
    private DcMotor intakeWheels;
    private DcMotor intakeMill;
    private DcMotor shooter;
    private Servo trigger;

    //inti for shooter class
    public Shooter(HardwareMap hardwareMap)
    {
        intakeWheels = hardwareMap.get(DcMotor.class, "intakeWheels");
        intakeMill = hardwareMap.get(DcMotor.class, "intakeMill");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        trigger = hardwareMap.get(Servo.class, "trigger");

        intakeWheels.setDirection(DcMotor.Direction.FORWARD);
        intakeMill.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        intakeWheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMill.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mechanicalDriveOutreach = new MechanicalDriveOutreach(hardwareMap);
    }

    public void controlMethod(Gamepad gamepad)
    {
        intake(gamepad);
        fire(gamepad);
    }

    private void intake(Gamepad gamepad)
    {
        if (gamepad.a)
        {
            mechanicalDriveOutreach.stop();
            mechanicalDriveOutreach.inch();
            intakeWheels.setPower(1);
            intakeMill.setPower(1);
            try
            {
                Thread.sleep(250);
            }
            catch (InterruptedException e) {e.printStackTrace();}
            mechanicalDriveOutreach.stop();
            try
            {
                Thread.sleep(2250);
            }
            catch (InterruptedException e) {e.printStackTrace();}
        }
        else if (gamepad.right_bumper)
        {
            intakeWheels.setPower(-1);
            intakeMill.setPower(-1);
        }
        else
        {
            intakeWheels.setPower(0);
            intakeMill.setPower(0);
            intakeWheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMill.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    }

    private void fire(Gamepad gamepad)
    {
        if (gamepad.y)
        {
            mechanicalDriveOutreach.stop();
            shooter.setPower(1);
            trigger.setPosition(1);
            try
            {
                Thread.sleep(1000);
            } catch (InterruptedException e) {e.printStackTrace();}
            trigger.setPosition(0);
            try
            {
                Thread.sleep(1000);
            } catch (InterruptedException e) {e.printStackTrace();}
            shooter.setPower(0);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    }

}

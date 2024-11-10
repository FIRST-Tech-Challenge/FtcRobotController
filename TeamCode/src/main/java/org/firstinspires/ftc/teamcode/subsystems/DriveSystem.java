package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.routines.Routine;

public class DriveSystem extends Subsystem
{

    private DcMotor drive_motor_fl;
    private DcMotor drive_motor_fr;
    private DcMotor drive_motor_bl;
    private DcMotor drive_motor_br;

    public DcMotor getDrive_motor_fl()
    {
        return drive_motor_fl;
    }

    public DcMotor getDrive_motor_fr()
    {
        return drive_motor_fr;
    }

    public DcMotor getDrive_motor_bl()
    {
        return drive_motor_bl;
    }

    public DcMotor getDrive_motor_br()
    {
        return drive_motor_br;
    }

    public void driveOnTick(double x, double y, double rx) {
        y = -y;

        getDrive_motor_fl().setPower(-(y + x + rx));
        getDrive_motor_fr().setPower(-(y - x + rx));
        getDrive_motor_bl().setPower((y - x - rx));
        getDrive_motor_br().setPower((y + x - rx));
    }

    public DriveSystem(Routine routine)
    {
        super(routine);
        drive_motor_fl = routine.hardwareMap.get(DcMotor.class, "drive_motor_fl");
        drive_motor_fr = routine.hardwareMap.get(DcMotor.class, "drive_motor_fr");
        drive_motor_bl = routine.hardwareMap.get(DcMotor.class, "drive_motor_bl");
        drive_motor_br = routine.hardwareMap.get(DcMotor.class, "drive_motor_br");

        drive_motor_fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_motor_bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_motor_fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_motor_br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive_motor_fr.setDirection(DcMotorSimple.Direction.REVERSE);
        drive_motor_bl.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}

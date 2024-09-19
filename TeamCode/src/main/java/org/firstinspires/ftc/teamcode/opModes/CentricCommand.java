package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.MechanicalDriveBase;
import org.firstinspires.ftc.teamcode.IMU.IMUControl;
import org.firstinspires.ftc.teamcode.fieldCentric.CentricDrive;
import org.firstinspires.ftc.teamcode.fieldCentric.TurnToHeading;
import org.firstinspires.ftc.teamcode.util.PIDController;

@TeleOp(name = "Centric Command", group = "FieldCentric")
public class CentricCommand extends OpMode
{
    MechanicalDriveBase mechanicalDriveBase;
    TurnToHeading turnToHeading;
    CentricDrive centricDrive;
    IMUControl imu;

    PIDController pid;

    @Override
    public void init()
    {
        mechanicalDriveBase = new MechanicalDriveBase(hardwareMap);
        imu = new IMUControl(hardwareMap, telemetry);
        turnToHeading = new TurnToHeading(telemetry, mechanicalDriveBase, imu);
        centricDrive = new CentricDrive(mechanicalDriveBase, telemetry);
    }

    @Override
    public void loop()
    {
        //centricDrive.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, imu.getAngle(), turnToHeading.turnToHeading(gamepad1.right_stick_x, gamepad1.right_stick_y, 0.15, 0.15));
        centricDrive.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, imu.getAngle(), gamepad1.right_stick_x);
    }
}

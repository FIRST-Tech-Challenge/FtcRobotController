package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    HardwareMap hardwareMap;
   DcMotorEx frontLeft, backLeft, frontRight, backRight;
    public DriveTrain(String frontLeftName,String backLeftName, String frontRightName, String backRightName)
    {
        frontLeft = hardwareMap.get(DcMotorEx.class, frontLeftName);
        backLeft = hardwareMap.get(DcMotorEx.class, backLeftName);
        frontRight = hardwareMap.get(DcMotorEx.class, frontRightName);
        backRight = hardwareMap.get(DcMotorEx.class, backRightName);
    }

    public void drive(double forward, double strafe, double rotate)
    {
        frontLeft.setPower(forward + strafe + rotate);
        backLeft.setPower(forward - strafe + rotate);
        frontRight.setPower(forward - strafe - rotate);
        backRight.setPower(forward + strafe - rotate);
    }

    public void cubedDrive(double forward, double strafe, double rotate)
    {
        frontLeft.setPower(Math.pow(forward + strafe + rotate, 3));
        backLeft.setPower(Math.pow(forward - strafe + rotate, 3));
        frontRight.setPower(Math.pow(forward - strafe - rotate, 3));
        backRight.setPower(Math.pow(forward + strafe - rotate, 3));
    }



}

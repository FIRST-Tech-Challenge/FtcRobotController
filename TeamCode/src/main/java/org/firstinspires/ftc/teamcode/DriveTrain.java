package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
public abstract class DriveTrain {

    public DcMotor[] driveMotors;
    public DriveTrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight){
         frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
         backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

         driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Drive Forward Test")
public class DriveForwardtest extends LinearOpMode {
    public void runOpMode(){
        DcMotor Left = hardwareMap.dcMotor.get("Left");
        DcMotor Right = hardwareMap.dcMotor.get("Right");

        Right.setDirection(DcMotorSimple.Direction.REVERSE);

        Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while(opModeIsActive()){
            Left.setPower(1);
            Right.setPower(1);
            sleep(3000);
            Left.setPower(0);
            Right.setPower(0);
        }
    }
}

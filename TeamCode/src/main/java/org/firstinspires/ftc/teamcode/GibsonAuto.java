package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="GibsonAUTO")
public class GibsonAuto extends LinearOpMode{
    // Definitions
    DcMotor m_frontLeft;
    DcMotor m_frontRight;
    DcMotor m_rearLeft;
    DcMotor m_rearRight;
    @Override
    public void runOpMode() {
        //Initialization code
        double LY = -gamepad1.left_stick_y; //y axis on left stick
        double LX = -gamepad1.left_stick_x; //X axis on left stick
        double RY = -gamepad1.right_stick_y; //y axis on right stick
        double RX = -gamepad1.right_stick_x; //X axis on right stick

        waitForStart();

        if(getRuntime() < 1){
            m_rearLeft.setPower(1);
            m_rearRight.setPower(1);
            m_frontLeft.setPower(1);
            m_frontRight.setPower(1);
        }
        else if(getRuntime() < 2){
            m_rearLeft.setPower(-1);
            m_rearRight.setPower(-1);
            m_frontLeft.setPower(-1);
            m_frontRight.setPower(-1);
        }
        if(getRuntime() < 3){
            m_rearLeft.setPower(1);
            m_rearRight.setPower(1);
            m_frontLeft.setPower(1);
            m_frontRight.setPower(1);
        }
        else if(getRuntime() < 5){
            m_rearLeft.setPower(-1);
            m_rearRight.setPower(-1);
            m_frontLeft.setPower(-1);
            m_frontRight.setPower(-1);
        }
        else{
            m_rearLeft.setPower(0);
            m_rearRight.setPower(0);
            m_frontLeft.setPower(0);
            m_frontRight.setPower(0);
        }
    }
}
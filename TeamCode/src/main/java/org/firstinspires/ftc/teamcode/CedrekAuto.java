package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;




@Autonomous(name="Cedrek Autonomous")
public class CedrekAuto extends LinearOpMode {
    DcMotor m_frontLeft;
    DcMotor m_frontRight;
    DcMotor m_rearLeft;
    DcMotor m_rearRight;

    // Definitions

    @Override
    public void runOpMode() {
        m_frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        m_frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        m_rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        m_rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        m_frontLeft.setDirection(DcMotor.Direction.REVERSE);
        m_rearLeft.setDirection(DcMotor.Direction.REVERSE);
        m_frontRight.setDirection(DcMotor.Direction.FORWARD);
        m_rearRight.setDirection(DcMotor.Direction.FORWARD);
        //Initialization code
        waitForStart();

        while(opModeIsActive()) {
            if (getRuntime() < 1) {
                m_frontLeft.setPower(-1);
                m_frontRight.setPower(-1);
                m_rearLeft.setPower(-1);
                m_rearRight.setPower(-1);


            } else if (getRuntime() < 2) {
                m_rearRight.setPower(1);
                m_rearLeft.setPower(1);
                m_frontRight.setPower(1);
                m_frontLeft.setPower(1);
            } else if (getRuntime() < 4) {
                m_frontLeft.setPower(-1);
                m_frontRight.setPower(-1);
                m_rearLeft.setPower(-1);
                m_rearRight.setPower(-1);

            } else if (getRuntime() < 6) {
                m_rearRight.setPower(1);
                m_rearLeft.setPower(1);
                m_frontRight.setPower(1);
                m_frontLeft.setPower(1);
            } else {
                m_frontLeft.setPower(0);
                m_rearRight.setPower(0);
                m_rearLeft.setPower(0);
                m_frontRight.setPower(0);
            }

        }






    }
}

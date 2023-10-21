package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Jaylen And Hody")
public class JaylenAndHody extends LinearOpMode{
    // Definitions
    DcMotor m_rearLeft;
    DcMotor m_rearRight;
    DcMotor m_frontLeft;
    DcMotor m_frontRight;


    @Override
    public void runOpMode() {
        //Initialization code
        waitForStart();
        m_frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        m_frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        m_rearLeft = hardwareMap.get(DcMotor.class,"rearLeft");
        m_rearRight = hardwareMap.get(DcMotor.class,"rearRight");

        m_rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        m_frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        m_rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        m_frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("status","initialization");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double y = .85*-gamepad1.left_stick_y;
            double x = .85*gamepad1.left_stick_x;
            double rotation = .75*gamepad1.right_stick_x;


            m_frontLeft.setPower(y + x + rotation);
            m_rearLeft.setPower(y - x + rotation);
            m_frontRight.setPower(y - x - rotation);
            m_rearRight.setPower(y + x - rotation);


        }
    }
}

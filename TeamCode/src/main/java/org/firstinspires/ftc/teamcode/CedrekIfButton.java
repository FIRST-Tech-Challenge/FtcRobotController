package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Basic Mecanum OpMode Cedrek", group="Basic")
public class CedrekIfButton extends LinearOpMode{
    DcMotor m_frontLeft;
    DcMotor m_frontRight;
    DcMotor m_rearLeft;
    DcMotor m_rearRight;

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


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;// Note: pushing stick forward gives negative value
            double x = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;


            /*m_rearLeft.setPower(y + x + rotation); // Note: pushing stick forward gives negative value
            m_rearLeft.setPower(y - x + rotation);
            m_frontRight.setPower(y - x - rotation);
            m_rearRight.setPower(y + x - rotation); */


            if(gamepad1.a){
                //Forward
                m_frontLeft.setPower(-1);
                m_frontRight.setPower(-1);
                m_rearLeft.setPower(-1);
                m_rearRight.setPower(-1);

            }
            else if (gamepad1.b){
                m_frontLeft.setPower(1);
                m_frontRight.setPower(-1);
                m_rearLeft.setPower(-1);
                m_rearRight.setPower(1);

            }
            else if (gamepad1.x){
                m_frontLeft.setPower(-1);
                m_frontRight.setPower(1);
                m_rearLeft.setPower(1);
                m_rearRight.setPower(-1);

            }
            else if (gamepad1.y){
                m_frontLeft.setPower(1);
                m_frontRight.setPower(1);
                m_rearLeft.setPower(1);
                m_rearRight.setPower(1);

            }
            else{
                m_frontLeft.setPower(0);
                m_frontRight.setPower(0);
                m_rearLeft.setPower(0);
                m_rearRight.setPower(0);
            }



        }

    }

}

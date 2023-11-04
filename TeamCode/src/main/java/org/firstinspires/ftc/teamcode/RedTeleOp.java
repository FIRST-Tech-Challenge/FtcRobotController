package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="RedTeleOp", group="Basic")
public class RedTeleOp extends LinearOpMode{
    DcMotor m_frontLeft;
    DcMotor m_frontRight;
    DcMotor m_rearLeft;
    DcMotor m_rearRight;
    Servo m_Servo;
    DcMotor m_climber;

    @Override
    public void runOpMode() {
        m_frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        m_frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        m_rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        m_rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        m_climber = hardwareMap.get(DcMotor.class,"Climber");
        m_Servo = hardwareMap.get(Servo.class,"Servo");

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
            double rotation = Math.pow(gamepad1.right_stick_x,3)/1.5;

            m_frontLeft.setPower(y + x + rotation); // Note: pushing stick forward gives negative value
            m_rearLeft.setPower(y - x + rotation);
            m_frontRight.setPower(y - x - rotation);
            m_rearRight.setPower(y + x - rotation);
            m_climber.setPower(gamepad2.right_stick_y);

            if (gamepad2.a){
                m_Servo.setPosition(1);
            }
            else {
                m_Servo.setPosition(0.3);
            }



        }

    }

}
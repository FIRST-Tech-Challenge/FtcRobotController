package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="BlackTeleOp", group="Basic")
public class BlackTeleOp extends LinearOpMode{
    DcMotor m_frontLeft;
    DcMotor m_frontRight;
    DcMotor m_rearLeft;
    DcMotor m_rearRight;
    DcMotor m_intake;
    DcMotor m_elevator;
    Servo m_pixspinner;
    Servo m_pixgrabber;

    @Override
    public void runOpMode() {
        m_frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        m_frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        m_rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        m_rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        m_intake = hardwareMap.get(DcMotor.class,"intake");
        m_elevator = hardwareMap.get(DcMotor.class,"elevator");
        m_pixspinner = hardwareMap.get(Servo.class, "pixelSpinner");
        m_pixgrabber = hardwareMap.get(Servo.class, "pixelGrabber");

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

            m_frontLeft.setPower(y + x + rotation); // Note: pushing stick forward gives negative value
            m_rearLeft.setPower(y - x + rotation);
            m_frontRight.setPower(y - x - rotation);
            m_rearRight.setPower(y + x - rotation);
            m_intake.setPower(gamepad2.left_trigger + -gamepad2.right_trigger);
            m_elevator.setPower(-gamepad2.left_stick_y);

            if (gamepad2.left_bumper){
                m_pixspinner.setPosition(.5);
            }
            else {
                m_pixspinner.setPosition(0);
            }

            if (gamepad2.a){
                m_pixgrabber.setPosition(.5);
            }
            else {
                m_pixgrabber.setPosition(0);
            }



        }

    }

}
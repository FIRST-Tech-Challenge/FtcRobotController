package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="Basic Mecanum OpMode Gibson", group="Basic")
public class gibsonmech extends LinearOpMode{
    DcMotor m_frontLeft;
    DcMotor m_frontRight;
    DcMotor m_rearLeft;
    DcMotor m_rearRight;
    ColorSensor color_sensor;

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
        telemetry.addData("please open README.Scam with your IP address visibly on your screen!!!", 0);
        telemetry.addData("Status", "Started");
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*double y = -gamepad1.left_stick_y;// Note: pushing stick forward gives negative value
            double x = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;

           /* m_frontLeft.setPower(y + x + rotation); // Note: pushing stick forward gives negative value
            m_rearLeft.setPower(y - x + rotation);
            m_frontRight.setPower(y - x - rotation);
            m_rearRight.setPower(y + x - rotation);*/
            double LY = -gamepad1.left_stick_y; //y axis on left stick
            double LX = -gamepad1.left_stick_x; //X axis on left stick
            double RY = -gamepad1.right_stick_y; //y axis on right stick
            double RX = -gamepad1.right_stick_x; //X axis on right stick

            if(LY == 1){
                m_rearLeft.setPower(1);
                m_rearRight.setPower(1);
                m_frontLeft.setPower(1);
                m_frontRight.setPower(1);
            }
            else if(LX == -1){
                m_rearLeft.setPower(1);
                m_rearRight.setPower(-1);
                m_frontLeft.setPower(-1);
                m_frontRight.setPower(1);
            }
            else if(LX == 1){
                m_rearLeft.setPower(-1);
                m_rearRight.setPower(1);
                m_frontLeft.setPower(1);
                m_frontRight.setPower(-1);
            }
            else if(LY == -1){
                m_rearLeft.setPower(-1);
                m_rearRight.setPower(-1);
                m_frontLeft.setPower(-1);
                m_frontRight.setPower(-1);
            }
            else if(RY == 1){
                m_rearLeft.setPower(1);
                m_rearRight.setPower(-1);
                m_frontLeft.setPower(1);
                m_frontRight.setPower(-1);
            }
            else if(RY == -1){
                m_rearLeft.setPower(-1);
                m_rearRight.setPower(1);
                m_frontLeft.setPower(-1);
                m_frontRight.setPower(1);
            }
            else if(gamepad1.right_trigger == 1){
                m_frontLeft.setPower(-1);
                m_frontRight.setPower(1);
                m_rearLeft.setPower(0);
                m_rearRight.setPower(0);
            }
            else if(gamepad1.left_trigger == 1){
                m_frontLeft.setPower(1);
                m_frontRight.setPower(-1);
                m_rearLeft.setPower(0);
                m_rearRight.setPower(0);
            }
            else{
                m_rearLeft.setPower(0);
                m_rearRight.setPower(0);
                m_frontLeft.setPower(0);
                m_frontRight.setPower(0);
            }
            if (color_sensor.alpha() < 20) {
                telemetry.addData("White color detected!", 0);
            }
            else{
                telemetry.addData("Other color detected!", 0);
            }

        }

    }

}

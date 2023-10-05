package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name=" Hody Auto", group="Basic")
public class HodyAuto extends LinearOpMode{

    void driveForward() {m_frontLeft.setPower(1);
                         m_frontLeft.setPower(1);
                         m_rearLeft.setPower(1);
                         m_rearRight.setPower(1);

    }
    void driveBackward() {
        m_frontLeft.setPower(-1);
        m_rearRight.setPower(-1);
        m_frontRight.setPower(-1);
        m_rearLeft.setPower(-1);
    }
    void stopRobot()  {
        m_frontLeft.setPower(0);
        m_rearLeft.setPower(0);
        m_frontRight.setPower(0);
        m_rearRight.setPower(0);
    }


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

            if (getRuntime() < 1) {
                driveForward();
            } else if (getRuntime() < 2) {
                driveBackward();
            } else if (getRuntime() < 4){
                driveForward();
            }
            else if (getRuntime() < 6) {
                driveBackward();
            }
            else{stopRobot();

            }
        }



    }




}

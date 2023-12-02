package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BasicAutonomous extends LinearOpMode
{

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    static final double FORWARD_SPEED = 0.6;
    static final double STRAFE_SPEED  = 0.5;

    static final String teamColor = "red"; // "red" or "blue"
    static final String startPos = "close"; // "close" is closer to scoreboard than "far"
    static final String parkPos = "close"; // "close" is closer to side edge than "far"

    private ElapsedTime runtime = new ElapsedTime();

    private void moveSideways()
    {
        double moveDir;
        if (teamColor == "blue")
        {
            moveDir = 1;
        }
        else
        {
            moveDir = -1;
        }

        double time;
        if (parkPos == "close")
        {
            time = 0.15;
        }
        else
        {
            time = 0.4;
        }

        leftFront.setPower(STRAFE_SPEED * moveDir);
        leftBack.setPower(STRAFE_SPEED * -(moveDir));
        rightFront.setPower(STRAFE_SPEED * -(moveDir));
        rightBack.setPower(STRAFE_SPEED + moveDir);

        while (opModeIsActive() && (runtime.seconds() < time))
        {
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void moveForward()
    {
        double time;
        if (startPos == "close")
        {
            time = 1.5;
        }
        else
        {
            time = 3.0;
        }

        leftFront.setPower(FORWARD_SPEED);
        leftBack.setPower(FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        rightBack.setPower(FORWARD_SPEED);

        while (opModeIsActive() && (runtime.seconds() < time))
        {
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();
        leftFront = hardwareMap.dcMotor.get("FLeft");
        leftBack = hardwareMap.dcMotor.get("BLeft");
        rightFront = hardwareMap.dcMotor.get("FRight");
        rightBack = hardwareMap.dcMotor.get("BRight");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        moveSideways();
        moveForward();
    }
}

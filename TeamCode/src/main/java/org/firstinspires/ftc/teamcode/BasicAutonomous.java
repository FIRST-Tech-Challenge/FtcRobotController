package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    private void moveSideways(String teamColor, String parkPos)
    {
        if (teamColor == "red")
        {
            double moveDir = 1;
        }
        else
        {
            double moveDir = -1;
        }

        if (parkPos == "close")
        {


            while (opModeIsActive() && (runtime.seconds() < time))
            {
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException
    {

    }
}

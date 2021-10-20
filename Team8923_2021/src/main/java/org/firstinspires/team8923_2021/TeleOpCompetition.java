package org.firstinspires.ftc.team8923_2020;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp Competition")

public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode()
    {
        initHardware();
        motorLift.setTargetPosition(0);
        waitForStart();

        while (opModeIsActive())
        {
            driveRobot();
            runintake();
            runShooter();
            runLift();
            runArm();
            runGrabber();
            runDriveSpeed();
            sendTelemetry();
            idle();
        }
    }
}

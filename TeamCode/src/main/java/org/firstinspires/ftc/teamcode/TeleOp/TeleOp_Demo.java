package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libs.AR.AR_Arm;
import org.firstinspires.ftc.teamcode.Libs.AR.MecanumDrive_5518;
//Anya testing push ability
@TeleOp(name = "Demo Bot TeleOp", group = "TeleOp")
public class TeleOp_Demo extends LinearOpMode
{
    private MecanumDrive_5518 mecanumDrive;

    private AR_Arm arm;

    //@Override
    public void runOpMode()
    {
        // Initialize the drivetrain
        mecanumDrive = new MecanumDrive_5518(this);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive())
        {
            // This call is made every loop and will read the current control pad values (for driving)
            // and update the drivetrain with those values.
            mecanumDrive.drive();

            //**************************************************************************************
            // ---------------------Gamepad 1 Controls ---------------------------------------------

            if (gamepad1.left_trigger != 0) {
                mecanumDrive.setBoost(1);
            }
            else {
                mecanumDrive.setBoost(0.5);
            }

//
            //**************************************************************************************
            //--------------------- TELEMETRY Code -------------------------------------------------
            // Useful telemetry data in case needed for testing and to find heading of robot
            mecanumDrive.getTelemetryData();

            telemetry.update();
        }
    }
}

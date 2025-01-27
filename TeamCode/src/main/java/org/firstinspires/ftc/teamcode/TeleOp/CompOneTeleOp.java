package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libs.AR.AR_Arm;
import org.firstinspires.ftc.teamcode.Libs.AR.MecanumDrive2;

@TeleOp(name = "CompOne TeleOp", group = "TeleOp")
public class CompOneTeleOp extends LinearOpMode
{
    private MecanumDrive2 mecanumDrive;

    private AR_Arm arm;

    //@Override
    public void runOpMode()
    {
        // Initialize the drivetrain
        mecanumDrive = new MecanumDrive2(this);
        arm = new AR_Arm(this);

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

            //**************************************************************************************
            // ---------------------Gamepad 2 Controls ---------------------------------------------
/*
            if (gamepad1.dpad_up) {
                telemetry.addData("Status","GP1:Square (setArmDeployPos) Light: Orange");

                // Set Arm into Deploy position.
                arm.setArmDeployPos();
//                light.customLight(AR_Light.GB_CLR_ORANGE);
            }
            if (gamepad1.dpad_right) {
                telemetry.addData("Status","GP1:Circle (setArmRestPos)");

                // Set Arm into Rest position.
                arm.setArmActivePos( );
//                light.customLight(AR_Light.GB_CLR_SAGE);
            }
            if (gamepad1.dpad_left) {
                telemetry.addData("Status","GP1:Cross (setArmGrabPos)");

                // Set Arm into GRAB position.
                arm.setArmGrabPos( );
//                light.customLight(AR_Light.GB_CLR_AZURE);
            }

            // Hotkeys (to change gripper position)
            if (gamepad2.left_trigger != 0) {
            } else if (gamepad2.right_trigger != 0) {
            }

            arm.updateArmPos();

*/
            //**************************************************************************************
            //--------------------- TELEMETRY Code -------------------------------------------------
            // Useful telemetry data in case needed for testing and to find heading of robot
            mecanumDrive.getTelemetryData();

            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libs.AR.Arm;
import org.firstinspires.ftc.teamcode.Libs.AR.MecanumDrive;

@TeleOp(name = "CompTwo TeleOp", group = "TeleOp")
public class CompTwoTeleOp extends LinearOpMode
{
    private MecanumDrive mecanumDrive;

    private Arm slide;

    //@Override
    public void runOpMode()
    {
        // Initialize the drivetrain
        mecanumDrive = new MecanumDrive(this);
        slide = new Arm (this);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive())
        {
            // This call is made every loop and will read the current control pad values (for driving)
            // and update the drivetrain with those values.
            mecanumDrive.drive();

            if (gamepad1.left_trigger != 0) {
                mecanumDrive.setBoost(1);
            }
            else {
                mecanumDrive.setBoost(0.5);
            }

            //**************************************************************************************
            // ---------------------Gamepad 2 Controls ---------------------------------------------


            if (gamepad2.y) {
                slide.openGripper();
            } else if (gamepad2.a){
                slide.closeGripper();
            }
            else{
                slide.guardGripper();
            }

            if (gamepad2.x) {
                slide.wristUp();
            } else if (gamepad2.b){
                slide.wristDown();
            }
            else{
                slide.wristGrab();
            }

            if (gamepad2.dpad_up){
                slide.moveSlideHigh();
            } else if (gamepad2.dpad_right) {
                slide.moveSlideMiddle();
            } else if (gamepad2.dpad_left) {
                slide.moveSlideLow();
            } else if (gamepad2.dpad_down) {
                slide.moveSlideDown();
            }


            // Todo: Remove when not needed anymore.
            //double forward = -gamepad1.left_stick_y;
            //double strafe = gamepad1.left_stick_x;
            //double rotation = gamepad1.right_stick_x;
            //mecanumDrive.move(strafe, forward, rotation);

            //**************************************************************************************
            //--------------------- TELEMETRY Code -------------------------------------------------

            // Useful telemetry data in case needed for testing and to find heading of robot
            mecanumDrive.getTelemetryData();
            slide.getTelemetryData();
            
            telemetry.update();
        }
    }
}


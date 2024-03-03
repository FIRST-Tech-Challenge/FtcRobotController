/// REGULA NR 1 - TOMA NU SCRIE COD PERFECT
package org.firstinspires.ftc.teamcode2024;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode2024.drive.SampleMecanumDrive;

@TeleOp(name="PuiuFaceDrifturiKaLaKaufland", group="Linear Opmode")
public class TeleOp2024 extends GlobalScope2024
{

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode()
    {
        Initialise();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(VolerLaPosition.currentPose);

        waitForStart();

        ct1 = new GamepadEx(gamepad1);
        ct2 = new GamepadEx(gamepad2);
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        IAMSPEED = new ButtonReader(ct1, GamepadKeys.Button.B);
        Launch = new ButtonReader(ct2, GamepadKeys.Button.B);
        ARMISSPEED = new ButtonReader(ct2, GamepadKeys.Button.A);
        Aerosmith = new ButtonReader(ct2, GamepadKeys.Button.RIGHT_BUMPER);
        Oasis = new ButtonReader(ct2, GamepadKeys.Button.LEFT_BUMPER);
        Hang = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        Fall = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);
        TurnToggle = new ButtonReader(ct1, GamepadKeys.Button.A);
        ArmUp = new ButtonReader(ct2, GamepadKeys.Button.DPAD_UP);
        ArmDown = new ButtonReader(ct2, GamepadKeys.Button.DPAD_DOWN);
        mb1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mb2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mb1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mb2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double arm_poz = 0.37;
       // SDrone.setPosition(arm_poz);
        SCutie.setPosition(0.37);
        //0.63 start
        int initbrat = 0;
        while (opModeIsActive())
        {
        /**
            if (gamepad1.dpad_left) {
                arm_poz += 0.01;
            }
            else if (gamepad1.dpad_right)
            {
                arm_poz -= 0.01;
            }

            arm_poz = Range.clip(arm_poz, 0, 1);
            SCutie.setPosition(arm_poz);
            telemetry.addData("cutie : ", arm_poz);
*/
            //* CEL MAI UTIL COD PE CARE L-AM SCRIS VREODATA
/**
            if (gamepad1.dpad_left)
                initbrat += 1;
            else if (gamepad1.dpad_right)
                initbrat -= 1;

            if (gamepad1.x)
            {
                mb1.setTargetPosition(initbrat);
                mb2.setTargetPosition(initbrat);
                mb1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mb2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mb1.setPower(0.2);
                mb2.setPower(0.2);
                while (mb1.isBusy() && mb2.isBusy() && !isStopRequested()) {
                    telemetry.addData("pozitia brat", mb1.getCurrentPosition());
                    telemetry.addData("pozitia brat2", mb2.getCurrentPosition());
                    telemetry.update();
                }
            }
//188
            telemetry.addData("pozitia brat", mb1.getCurrentPosition());
            telemetry.addData("pozitia brat2", mb2.getCurrentPosition());
            telemetry.addData("initbrat", initbrat);
            telemetry.update();


            if (gamepad1.dpad_left) {
              //    SHang.setPosition(arm_poz);
               arm_poz += 0.01;
            }
            else if (gamepad1.dpad_right) {
                 // SHang.setPosition(arm_poz);
                arm_poz -= 0.01;
            }
            arm_poz = Range.clip(arm_poz, 0, 1);
            SHang.setPosition(arm_poz);


           telemetry.addData("SDrome %.2f", SDrone.getPosition());
            telemetry.update();

            sleep(40);
            //currentHeading= drive.getExternalHeading();
 */
            drive.update();
            Pose2d positionestimee = drive.getPoseEstimate();

            telemetry.addData("x", positionestimee.getX());
            telemetry.addData("y", positionestimee.getY());
            telemetry.addData("heading", Math.toDegrees(positionestimee.getHeading()));
            telemetry.update();

          /**  TurnToggle.readValue();
            if (TurnToggle.wasJustPressed())
            {
                /**
                double LeHeading = Math.toDegrees(positionestimee.getHeading());
                if (0 <= LeHeading && LeHeading <= 90)
                    drive.turn(Math.toRadians(90 - LeHeading));
                else if (270 <= LeHeading && LeHeading <= 360)
                    drive.turn( Math.toRadians(90 + 360 - LeHeading));
                else if (90 <= LeHeading && LeHeading <= 180)
                    drive.turn(Math.toRadians(-(LeHeading - 90)));
                else if (180 <= LeHeading && LeHeading <= 270)
                    drive.turn(Math.toRadians(-(90 + LeHeading - 180)));
                 */
                
       //         drive.turnAsync(Angle.normDelta(Math.toRadians(90) - positionestimee.getHeading()));

        //   }

            WeGottaMove();
            WeGottaLift();
            WeGottaLunchDrone();
            WeGottaDoArmMovements();
            WeGottaKillOurselves();
            ToggleArmUpDown();


        }

    }
}
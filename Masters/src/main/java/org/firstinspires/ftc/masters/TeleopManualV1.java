package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.masters.components.DriveTrain;
import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outtake;

@Config // Enables FTC Dashboard
@TeleOp(name = "V1 Manual Teleop")
public class TeleopManualV1 extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double Blank = 0;

    public void runOpMode() throws InterruptedException {


        double servo1pos = 0.5;
        double servo2pos = 0.5;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Init init = new Init(hardwareMap);
        DriveTrain driveTrain = new DriveTrain(init, telemetry);
        Outtake outtake = new Outtake(init, telemetry);
        Intake intake = new Intake(init, telemetry);

        outtake.initTeleopWall();

        int target=0;

        int dpadUpPressed = 0;
        int dpadDownPressed = 0;

        boolean relased = true;

        telemetry.addData("Before", outtake.outtakeSlideEncoder.getCurrentPosition());

        telemetry.update();

        waitForStart();

        intake.transferIntake();
        intake.closeGate();


        while (opModeIsActive()) {

            driveTrain.driveNoMultiplier(gamepad1, DriveTrain.RestrictTo.XYT);

            if (gamepad1.right_stick_y > 0.5){
                intake.retractSlide();
            } else if (gamepad1.right_stick_y < -0.5) {
                intake.extendSlide();
            }

            if (gamepad1.right_stick_x > 0.1){
                intake.startIntake();
            } else if (gamepad1.right_stick_x < -0.1) {
                intake.reverseIntake();
            }

            if (gamepad1.right_stick_button) {
                intake.stopIntake();
            }

            if (gamepad1.x){
                //turn on
                intake.startIntake();

            } else if (gamepad1.y) {
//                reverse
                intake.reverseIntake();
            }

//            if (gamepad1.dpad_up) {
//                outtake.moveToBucket();
//            }
//
//            if (gamepad1.dpad_down) {
//                outtake.moveToTransfer();
//            }

            if (gamepad1.dpad_left) {
                outtake.moveToPickUpFromWall();

            }


            if (gamepad1.right_bumper && relased) {
                if (dpadUpPressed == 0) {
                    intake.extendSlideMax();
                    dpadDownPressed = 1;
                    dpadUpPressed=1;
                } else if (dpadUpPressed==1){
                    dpadDownPressed=0;
                    //intake.dropIntake();
                }
                relased = false;
            } else {
                relased = true;
            }


            if (gamepad1.left_bumper && relased) {
                if (dpadDownPressed == 0) {
                    //intake.moveIntakeToTransfer();
                    dpadDownPressed=1;
                    dpadUpPressed=1;
                } else if (dpadDownPressed==1){
                    intake.retractExtensionFully();;
                    dpadUpPressed=0;
                    dpadDownPressed=0;
                }
                relased = false;
            } else {
                relased=true;
            }

            if (gamepad1.dpad_right) {
                outtake.scoreSpecimen();
            }


            if(gamepad1.a){
                outtake.openClaw();
            } else if (gamepad1.b) {
                outtake.closeClaw();
            }

            if (gamepad1.dpad_up) {
                intake.intakeintake();
            }

            if (gamepad1.dpad_down) {
                intake.transferIntake();
            }

            // Controller 2 anti-fuck up code

            // Reset Vertical slides

            if (gamepad2.a){

            }

            // Adjust Slides

            // Reset Horizontal slides

            outtake.update();
            intake.update();


//            telemetry.addData("Slide Target", outtake.getTarget());
//            telemetry.addData("Before", outtake.outtakeSlideEncoder.getCurrentPosition());
//
//            telemetry.addData("Slide Position", outtake.getExtensionPos());
//            telemetry.addData("Slide Servo Pos", intake.getExtensionPosition());
//            telemetry.addData("Diffy Servo1 Pos", servo1pos);
//            telemetry.addData("Diffy Servo2 Pos", servo2pos);
//            telemetry.update();

        }
    }
}
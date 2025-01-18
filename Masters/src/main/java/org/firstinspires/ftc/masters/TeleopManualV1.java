package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        telemetry.addData("Before", outtake.outtakeSlideEncoder.getCurrentPosition());

        telemetry.update();

        waitForStart();


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
                intake.dropIntake();
            } else if (gamepad1.y) {
                intake.liftIntake();
            }

            if (gamepad1.dpad_up) {
                outtake.moveToBucket();
//                outake.diffy1(ITDCons.BucketDiffy1);
//                outake.diffy2(ITDCons.BucketDiffy2);
                //target= ITDCons.BucketTarget;
            }

            if (gamepad1.dpad_down) {
                outtake.moveToTransfer();
//                outake.diffy1(ITDCons.FloorDiffy1);
//                outake.diffy2(ITDCons.FloorDiffy2);
                target=0;
            }

            if (gamepad1.dpad_left) {
                outtake.moveToPickUpFromWall();

//                outake.diffy1(ITDCons.WallDiffy1);
//                outake.diffy2(ITDCons.WallDiffy2);
              //  target=0;
            }

            if (gamepad1.dpad_right) {
                outtake.scoreSpecimen();
//                outake.diffy1(ITDCons.SpecimenDiffy1);
//                outake.diffy2(ITDCons.SpecimenDiffy2);
               // target= ITDCons.SpecimenTarget;

            }

            if (gamepad1.left_bumper) {
//                if (target>100) {
//                    target = target - 100;
//                }
//                outake.diffy1(ITDCons.ReleaseDiffy1);
//                outake.diffy2(ITDCons.ReleaseDiffy2);
//                target= ITDCons.ReleaseTarget;
                outtake.releaseSpecimen();

            }

            if(gamepad1.a){
                outtake.openClaw();
            } else if (gamepad1.b) {
                outtake.closeClaw();
            }

            outtake.moveSlide();
            outtake.updateOuttake();


            telemetry.addData("Slide Target", outtake.getTarget());
            telemetry.addData("Slide Position", outtake.getExtensionPos());
            telemetry.addData("Slide Servo Pos", intake.getExtensionPosition());
            telemetry.addData("Diffy Servo1 Pos", servo1pos);
            telemetry.addData("Diffy Servo2 Pos", servo2pos);
            telemetry.update();

        }
    }
}


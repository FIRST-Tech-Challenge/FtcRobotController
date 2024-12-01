package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.masters.components.DriveTrain;
import org.firstinspires.ftc.masters.components.ITDCons;
import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outake;

@Config // Enables FTC Dashboard
@TeleOp(name = "V1 Manual Teleop")
public class TeleopManualV1 extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double Blank = 0;

    public void runOpMode() throws InterruptedException {

        int slideTarget = 0;

        double servo1pos = 0.5;
        double servo2pos = 0.5;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Init init = new Init(hardwareMap);
        DriveTrain driveTrain = new DriveTrain(init, telemetry);
        Outake outake = new Outake(init, telemetry);
        Intake intake = new Intake(init, telemetry);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            driveTrain.driveNoMultiplier(gamepad1, DriveTrain.RestrictTo.XYT);

            if (gamepad1.right_stick_y > 0.1){
                intake.retractSlide();
            } else if (gamepad1.right_stick_y < -0.1) {
                intake.extendSlide();
            }

            if (gamepad1.right_stick_x > 0.1){
                intake.intakePower(-.7);
            } else if (gamepad1.right_stick_x < -0.1) {
                intake.intakePower(.7);
            }

            if (gamepad1.right_stick_button) {
                intake.intakePower(0);
            }

            if (gamepad1.x){
                intake.intakeLift(ITDCons.liftDown);
            } else if (gamepad1.y) {
                intake.intakeLift(ITDCons.liftUp);
            }

//            if (gamepad1.right_bumper){
//                slideTarget = slideTarget + 5;
//            } else if (gamepad1.left_bumper) {
//                slideTarget = slideTarget - 5;
//            }
//            outake.moveSlide(slideTarget);

            if (gamepad1.dpad_up) {
                outake.diffy1(ITDCons.BucketDiffy1);
                outake.diffy2(ITDCons.BucketDiffy2);
                outake.moveSlide(ITDCons.BucketTarget);
            }

            if (gamepad1.dpad_down) {
                outake.diffy1(ITDCons.FloorDiffy1);
                outake.diffy2(ITDCons.FloorDiffy2);
                outake.moveSlide(0);
            }

            if (gamepad1.dpad_left) {
                outake.diffy1(ITDCons.WallDiffy1);
                outake.diffy2(ITDCons.WallDiffy2);
                outake.moveSlide(0);
            }

            if (gamepad1.dpad_right) {
                outake.diffy1(ITDCons.SpecimenDiffy1);
                outake.diffy2(ITDCons.SpecimenDiffy2);
                outake.moveSlide(ITDCons.SpecimenTarget);
            }

            if(gamepad1.a){
                outake.moveClaw(ITDCons.open);
            } else if (gamepad1.b) {
                outake.moveClaw(ITDCons.close);
            }


            telemetry.addData("Slide Target", outake.getTarget());
            telemetry.addData("Slide Position", outake.getExtensionPos());
            telemetry.addData("Slide Servo Pos", intake.getExtensionPosition());
            telemetry.addData("Diffy Servo1 Pos", servo1pos);
            telemetry.addData("Diffy Servo2 Pos", servo2pos);
            telemetry.update();

        }
    }
}


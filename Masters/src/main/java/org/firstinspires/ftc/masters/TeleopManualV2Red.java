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
import org.firstinspires.ftc.masters.components.Outtake;

@Config // Enables FTC Dashboard
@TeleOp(name = "V2 Manual Teleop Red")
public class TeleopManualV2Red extends LinearOpMode {


/*   controls:
    x : drop intake/ motor on/ auto spit out wrong color (maybe lift and back down)
    if yellow transfer (maybe retract first
    if red/blue put intake in neutral position. if slide full, retract to half, if half retract fully
    control: low bucket
    y: high bucket/score spec
    right stick y up: extends slide and spit out red/blue (to human player)
    b: go to wall position (if yellow in transfer go to low bucket), press again close claw
    control: yellow in transfer go high bucket else score spec
    a: open claw (go to transfer or wall)
    control: extendo half out
    control: extendo full out

    control: to neutral
    control to transfer

    2nd controller
    vertical slide reset
    extendo back in
    adjust height of score spec
    adjust angle spec scoring

*/

    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    public static double Blank = 0;

    public void runOpMode() throws InterruptedException {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Init init = new Init(hardwareMap);
        DriveTrain driveTrain = new DriveTrain(init, telemetry);
        Outtake outtake = new Outtake(init, telemetry);
        Intake intake = new Intake(init, telemetry);

        outtake.setIntake(intake);
        intake.setOuttake(outtake);
        intake.setAllianceColor(ITDCons.Color.red);

        int target=0;

        int dpadUpPressed = 0;
        int dpadDownPressed = 0;

        boolean leftDown = false, leftPressed = false;


        boolean bPressed = false;
        boolean relased = true;

        telemetry.addData("Before", outtake.outtakeSlideEncoder.getCurrentPosition());

        telemetry.update();

        waitForStart();

        intake.initStatusTeleop();
        outtake.initTeleopWall();


        while (opModeIsActive()) {

            driveTrain.driveNoMultiplier(gamepad1, DriveTrain.RestrictTo.XYT);

            if (gamepad1.right_stick_y > 0.5){
                intake.retractSlide();
            } else if (gamepad1.right_stick_y < -0.5) {
                // extends/ejects to human player
                intake.extendSlideHumanPlayer();
            }

            if (gamepad1.right_stick_x > 0.2){

            } else if (gamepad1.right_stick_x < -0.2) {
                intake.ejectIntake();
            }

            if (gamepad1.right_stick_button) {
                intake.stopIntake();
            }

            if (gamepad1.touchpad){
                intake.toTransfer();
            }

           if (gamepad1.dpad_up) {
                intake.extendSlideMax();
            } else if (gamepad1.dpad_down){
                intake.extendSlideHalf();
            }



            if(gamepad1.a){
                outtake.openClaw();
            }
            if (gamepad1.b) {
               if (!bPressed) {
                    bPressed = true;
                   if (outtake.isReadyToPickUp()) {
                       outtake.closeClaw();
                   } else if (intake.getColor() == ITDCons.Color.yellow) {
                       outtake.scoreSampleLow();
                   } else {
                       outtake.moveToPickUpFromWall();
                   }
               }
            } else {
                bPressed = false;
            }

            if (gamepad1.x){

            } else if (gamepad1.y){
                outtake.score();
            }

            if (gamepad1.right_bumper) {

                outtake.moveToTransfer();
            } else if (gamepad1.left_bumper){
                intake.toNeutral();
            }

            if (gamepad1.dpad_left && !leftDown) {
                leftPressed = !leftPressed;
                leftDown = true;
            } else if (!gamepad1.dpad_left) {
                leftDown = false;
            }
            if (leftPressed) {

                intake.pushOut();

            } else {

                intake.pushIn();

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
            telemetry.addData("intake color", intake.getColor());
            telemetry.addData("intake status", intake.getStatus());
            telemetry.addData("Outtake status", outtake.getStatus());

            telemetry.update();

        }
    }
}

// manual transfer or neutral
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.FSMBot;
import org.firstinspires.ftc.teamcode.sample.Sample;

@TeleOp(name = "Drive")
public class TeleOps extends LinearOpMode {
    private FSMBot robot = new FSMBot(this);

    private Sample lastSample = null;

    private ElapsedTime subtimer = new ElapsedTime();

    private boolean blueAlliance = true;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.isAuto = false;

        robot.init(hardwareMap);

        telemetry.addData("blue alliance:", blueAlliance);

        while (opModeInInit()) {
            if (gamepad1.dpad_right) {
                blueAlliance = !blueAlliance;
                telemetry.addData("blue alliance:", blueAlliance);
            }
            telemetry.update();
        }
        robot.currentState = FSMBot.gameState.PRE_DRIVE;
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("target slides", robot.sampleSlideDropOffPos);
            telemetry.addData("real target slides",robot.slideTarget);
//            if(gamepad1.y) {
//                robot.currentState = FSMBot.gameState.DRIVE;
            if(robot.currentState == FSMBot.gameState.DRIVE){
                robot.subIntake(gamepad1.a);
                robot.raisePivotSample(gamepad1.b);
                robot.pivotUpTimer.reset();
                if (gamepad1.x) {
                    robot.intake(true);
                }

            }

            if(robot.currentState == FSMBot.gameState.SUBMERSIBLE_INTAKE_3 && robot.subRetractTimer.milliseconds() > 400){
                robot.retractSubIntake(gamepad1.a);
            }
            if(robot.currentState == FSMBot.gameState.SAMPLE_SCORING_HIGH_1 && robot.pivotUpTimer.milliseconds() > 400){
                robot.raiseSlidesSample(gamepad1.b);
                robot.slidesUpTimer.reset();
            }
            if(robot.currentState == FSMBot.gameState.SAMPLE_SCORING_HIGH_2 && robot.slidesUpTimer.milliseconds() > 400){
                robot.slideControlVert(gamepad1.right_bumper, gamepad1.left_bumper);

                if(gamepad1.dpad_up||gamepad1.dpad_down){
                    robot.manualOverride(true);
                }
                if(gamepad1.b) {
                    robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_3;
                    robot.outtakeTimer.reset();
                }
            }
            robot.slideControlVert(gamepad1.right_bumper, gamepad1.left_bumper);

            // above is all gamepad 1 and sample stuff
            if(robot.currentState == FSMBot.gameState.DRIVE && robot.hangTimer.milliseconds() > 500){
                if(gamepad2.x){
                    robot.currentState = FSMBot.gameState.HANG_UP;
                    robot.hangTimer.reset();
                }
            }
            if(robot.currentState == FSMBot.gameState.HANG_UP && robot.hangTimer.milliseconds() > 500){
                if(gamepad2.x){
                    robot.currentState = FSMBot.gameState.HANG_DOWN;
                    robot.hangTimer.reset();
                }
            }
            // above is hang 1 on gamepad 2
            robot.onLoop(0, "manual drive");


            telemetry.setMsTransmissionInterval(11);
            //hand drive
            robot.driveByHandFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x*-0.65, (gamepad1.right_trigger > 0.5), gamepad2.left_stick_x,
                    gamepad2.left_stick_y, gamepad2.right_stick_x*-0.65, (gamepad2.right_trigger > 0.5));
            if(robot.currentState == FSMBot.gameState.SUBMERSIBLE_INTAKE_2) {
                robot.slideControl(gamepad1.right_bumper, gamepad1.left_bumper);
            }
//            robot.pivotControl(gamepad1.dpad_left, gamepad1.dpad_right);
            //specimen stuff on gamepad2
            if(gamepad2.b){
                robot.currentState = FSMBot.gameState.SPECIMEN_SCORING_HIGH_DRIVE;
            }
            if(gamepad1.left_stick_button){
                robot.currentState = FSMBot.gameState.PRE_DRIVE;
            }
//            robot.intake(gamepad1.a);
            if(robot.currentState == FSMBot.gameState.SUBMERSIBLE_INTAKE_3 || robot.currentState == FSMBot.gameState.DRIVE) {
                robot.outake(gamepad1.x);
            }

            if(gamepad1.left_trigger > 0.5){
                robot.manualOverride = true;
            }
            if(gamepad1.y){
                robot.resetAngle(true);
            }



            if(gamepad2.dpad_up){
                robot.pitch(1);
            }
            if(gamepad2.dpad_down){
                robot.pitch(-1);
 telemetry.addData("Current roll", robot.getCurrentPitch());
            telemetry.addData("Current pitch", robot.getCurrentRoll());            }

            if(gamepad2.dpad_right){
                robot.roll(1);
            }
            if (gamepad2.dpad_left){
                robot.roll(-1);
            }

            //to take specimen and go into scoring position
            if (gamepad2.a && robot.currentState != FSMBot.gameState.WALL_INTAKE_1) {
                robot.currentState = FSMBot.gameState.WALL_INTAKE_1;
                robot.wallIntakeTimer.reset();            }
            else if(gamepad2.a && robot.wallIntakeTimer.milliseconds() > 400){
                robot.currentState = FSMBot.gameState.DRIVE;
            }

            //after scoring specimen, lower back into scoring position
            if (gamepad2.b && robot.currentState == FSMBot.gameState.WALL_INTAKE_1) {
                robot.currentState = FSMBot.gameState.SPECIMEN_SCORING_HIGH_MANUAL1;
                robot.submersibleTimer.reset();
            }
            if (gamepad2.b && robot.currentState == FSMBot.gameState.SPECIMEN_SCORING_HIGH_MANUAL1 && robot.submersibleTimer.milliseconds() > 400) {
                robot.currentState = FSMBot.gameState.SPECIMEN_SCORING_HIGH_MANUAL2;
            }



            if (robot.pivotOutOfRange) {

                telemetry.addData("ERROR", "Pivot is out of Range");

            }
            telemetry.addData("state:",robot.currentState);
            telemetry.addData("slide pos" ,robot.getSlidePosition());
            telemetry.addData("pivot pos" ,robot.getPivotPosition());
            telemetry.addData("pivot Target" ,robot.pivotTarget);
            telemetry.addData("Current roll", robot.getCurrentPitch());
            telemetry.addData("Current pitch", robot.getCurrentRoll());
            telemetry.addData("Manual Override:", robot.getOverride());

        }
        robot.close();
    }

}

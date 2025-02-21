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
            if(robot.currentState == FSMBot.gameState.SUBMERSIBLE_INTAKE_2 && robot.subRetractTimer.milliseconds() > 130){
                robot.retractSubIntake(gamepad1.a);
            }
            if(robot.currentState == FSMBot.gameState.SAMPLE_SCORING_HIGH_1 && robot.pivotUpTimer.milliseconds() > 130){
                robot.raiseSlidesSample(gamepad1.b);
                robot.slidesUpTimer.reset();
            }
            if(robot.currentState == FSMBot.gameState.SAMPLE_SCORING_HIGH_2 && robot.slidesUpTimer.milliseconds() > 130){
                if(gamepad1.dpad_up){
                    robot.setSlidePos(770);
                }
                if(gamepad1.b) {
                    robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_3;
                    robot.outtakeTimer.reset();
                }
            }
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

            robot.onLoop(0, "manual drive");


            telemetry.setMsTransmissionInterval(11);

            robot.driveByHandFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x*0.7, gamepad1.left_stick_button, gamepad2.left_stick_x,
                    gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.left_stick_button);
            robot.slideControl(gamepad1.dpad_right, gamepad1.dpad_left);
            robot.pivotControl(gamepad1.dpad_up, gamepad1.dpad_down);
            if(gamepad2.b){
                robot.currentState = FSMBot.gameState.SPECIMEN_SCORING_HIGH;
            }
            if(gamepad1.left_stick_button){
                robot.currentState = FSMBot.gameState.PRE_DRIVE;
            }
//            robot.intake(gamepad1.a);
            if(robot.currentState == FSMBot.gameState.SUBMERSIBLE_INTAKE_2 || robot.currentState == FSMBot.gameState.DRIVE) {
                robot.outake(gamepad1.x);
            }



            if(gamepad1.x){
                robot.currentState = FSMBot.gameState.SUBMERSIBLE_INTAKE_1;
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
            if (gamepad2.left_bumper){
                robot.wallIntakeDone = true;
            } else {
                robot.wallIntakeDone = false;
            }

            //after scoring specimen, lower back into scoring position
            if (gamepad2.right_bumper) {
                robot.specimenScored = true;
            } else {
                robot.specimenScored = false;
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

        }
        robot.close();
    }

}

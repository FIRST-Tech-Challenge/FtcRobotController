package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bots.FSMBot;
import org.firstinspires.ftc.teamcode.bots.RollerIntakeBot;
import org.firstinspires.ftc.teamcode.sample.Sample;

@TeleOp(name = "Drive")
public class TeleOps extends LinearOpMode {
    private FSMBot robot = new FSMBot(this);

    private Sample lastSample = null;

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

        waitForStart();

        while(opModeIsActive()){
            robot.currentState = FSMBot.gameState.DRIVE;
            robot.onLoop(0, "manual drive");
            telemetry.setMsTransmissionInterval(11);
            robot.driveByHandFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x*0.7, gamepad1.left_stick_button, gamepad2.left_stick_x,
                    gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.left_stick_button);
            robot.slideControl(gamepad1.dpad_right, gamepad1.dpad_left);
            robot.pivotControl(gamepad1.dpad_up, gamepad1.dpad_down);
            robot.intake(gamepad1.a);
            robot.outake(gamepad1.b);
            if(gamepad1.left_bumper){
                robot.currentState = FSMBot.gameState.SUBMERSIBLE_INTAKE_1;
            }

            if(gamepad2.dpad_up){
                robot.pitch(1);
            }
            if(gamepad2.dpad_down){
                robot.pitch(-1);
            }
            if(gamepad2.dpad_right){
                robot.roll(1);
            }
            if (gamepad2.dpad_left){
                robot.roll(-1);
            }

            //to take specimen and go into scoring position
            if (gamepad1.left_bumper){
                robot.wallIntakeDone = true;
            } else {
                robot.wallIntakeDone = false;
            }

            //after scoring specimen, lower back into scoring position
            if (gamepad1.right_bumper) {
                robot.specimenScored = true;
            } else {
                robot.specimenScored = false;
            }

            telemetry.addData("state:",robot.currentState);

            if (robot.pivotOutOfRange) {

                telemetry.addData("ERROR", "Pivot is out of Range");

            }

        }
        robot.close();
    }

}

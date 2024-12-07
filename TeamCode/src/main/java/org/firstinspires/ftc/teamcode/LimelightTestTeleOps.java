package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bots.HangBot;
import org.firstinspires.ftc.teamcode.sample.Sample;

@TeleOp(name = "Limlight Tester")
public class LimelightTestTeleOps extends LinearOpMode {
    private HangBot robot = new HangBot(this);

    private Sample lastSample = null;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.isAuto = false;

        robot.init(hardwareMap);


        robot.switchPipeline(1);
        telemetry.addData("pipeline: 1", null);

        while (opModeInInit()) {

            if (gamepad1.dpad_right) {

                robot.switchPipeline(1);
                telemetry.addData("pipeline", "red");

            }
            if (gamepad1.dpad_left) {

                robot.switchPipeline(2);
                telemetry.addData("pipeline", "blue");

            }
            telemetry.update();

        }

        waitForStart();
        while(opModeIsActive()){


            telemetry.setMsTransmissionInterval(11);

            robot.resetAngle(gamepad1.x);

            robot.onLoop(0, "manual drive");

            robot.pivotControl(gamepad1.dpad_up, gamepad1.dpad_down);
            robot.slideControl(gamepad1.right_bumper, gamepad1.left_bumper);
            robot.pinchControl(gamepad1.a, gamepad1.b);
            robot.rotateControl(gamepad1.left_trigger > 0.5,gamepad1.right_trigger > 0.5);
//            robot.scoreSpecimen(gamepad2.y);
            robot.hang(gamepad2.x);
            // limelight detection check
            if (gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up) {
                robot.inAutoPickup = false;
                robot.openPinch();
                robot.rotateToVerticalPos();
                robot.pivotToSearchPos();
                robot.moveSlideToSearchPos();
                boolean isBlueAlliance, includeSharedSample;
                if (gamepad2.dpad_down) {
                    isBlueAlliance = false;
                    includeSharedSample = true;
                }
                else if (gamepad2.dpad_left) {
                    isBlueAlliance = true;
                    includeSharedSample = false;
                }
                else if (gamepad2.dpad_right) {
                    isBlueAlliance = false;
                    includeSharedSample = false;
                }
                else {
                    isBlueAlliance = true;
                    includeSharedSample = true;
                }
                Sample s = robot.detectOne(isBlueAlliance, includeSharedSample, telemetry);
                lastSample = s;
            }
            if (gamepad2.a){
                robot.strafing(0.1);
            }
            if (gamepad2.b){
                robot.strafing(-0.1);
            }
            if (gamepad2.left_bumper){
                robot.pickup(true, true, false, telemetry);
            }
            else if (gamepad2.right_bumper){
                robot.pickup(false, true, false, telemetry);
            } else {
                robot.driveByHandFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y,
                        gamepad1.right_stick_x*0.7, gamepad1.left_stick_button, gamepad2.left_stick_x,
                        gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.left_stick_button);

            }
//            if (gamepad2.x) {
//                robot.rotateToVerticalPos();
//            }

            if (lastSample != null) {
                telemetry.addData("detected : ", lastSample.toString());
            } else {
                telemetry.addData("nothing detected : ", "null");
            }
//                telemetry.addData("limelight", robot.limelight.getConnectionInfo());
            telemetry.addData("limelight.status", robot.limelight.getStatus());

            telemetry.addData("slide position", robot.getSlidePosition());
            telemetry.addData("pivot position", robot.getPivotPosition());
            telemetry.addData("rotate position", robot.rotate.getPosition());
            telemetry.addData("vR",robot.rightFront.getCurrentPosition() );
            telemetry.addData("vL", robot.rightRear.getCurrentPosition());
            telemetry.addData("h", robot.leftRear.getCurrentPosition());

            if (robot.pivotOutOfRange) {

                telemetry.addData("ERROR", "Pivot is out of Range");

            }
            telemetry.addData("hang position", robot.getCurrentPositionString());



        }
        robot.close();
    }

}

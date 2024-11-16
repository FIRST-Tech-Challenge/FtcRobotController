package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.LimelightBot;
import org.firstinspires.ftc.teamcode.bots.PivotBot;
import org.firstinspires.ftc.teamcode.bots.PinchBot;

@TeleOp(name = "Drive")
public class TeleOps extends LinearOpMode {
    private PinchBot robot = new PinchBot(this);

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

            robot.driveByHandFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x*0.7, gamepad1.left_stick_button, gamepad2.left_stick_x,
                    gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.left_stick_button);

            robot.resetAngle(gamepad1.x);

            robot.onLoop(0, "manual drive");

            robot.pivotControl(gamepad1.dpad_up, gamepad1.dpad_down);
            robot.slideControl(gamepad1.dpad_left, gamepad1.dpad_right);
            robot.pinchControl(gamepad1.a, gamepad1.b);
            robot.rotateControl(gamepad1.left_trigger > 0.5,gamepad1.right_trigger > 0.5);

            double[] values = robot.detectOne();

            telemetry.addData("angle: ", values[0]);

            telemetry.addData("slide position", robot.getSlidePosition());
            telemetry.addData("pivot position", robot.getPivotPosition());
            telemetry.addData("rotate position", robot.rotate.getPosition());

            if (robot.pivotOutOfRange) {

                telemetry.addData("ERROR", "Pivot is out of Range");

            }

            telemetry.update();



        }
        robot.close();
    }

}

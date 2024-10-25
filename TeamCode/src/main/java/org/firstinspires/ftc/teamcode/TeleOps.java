package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.LimelightBot;
import org.firstinspires.ftc.teamcode.bots.PinchBot;

@TeleOp(name = "Drive")
public class TeleOps extends LinearOpMode {
    private PinchBot robot = new PinchBot(this);

    @Override
    public void runOpMode() throws InterruptedException {

        robot.isAuto = false;

        robot.init(hardwareMap);

        waitForStart();
        while(opModeIsActive()){
            telemetry.setMsTransmissionInterval(11);

            robot.driveByHandFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x*0.7, gamepad1.left_stick_button, gamepad2.left_stick_x,
                    gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.left_stick_button);

            robot.resetAngle(gamepad1.x);

            robot.onLoop(0, "manual drive");
            robot.PivotControl(gamepad1.dpad_up, gamepad1.dpad_down);
            robot.SlideControl(gamepad1.dpad_left, gamepad1.dpad_right);

            telemetry.addData("slide position: ", robot.Slide.getCurrentPosition());
            telemetry.update();
            robot.detect(gamepad1.a);

        }
        robot.close();
    }

}

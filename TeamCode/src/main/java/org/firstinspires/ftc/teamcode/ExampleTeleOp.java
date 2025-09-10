package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.aim.components.Button;

@TeleOp
public class ExampleTeleOp extends LinearOpMode{
    private Robot robot = new Robot();
    private Button testButton = new Button();

    @Override
    public void runOpMode() {
        robot.init(this);

        waitForStart();

        robot.start();

        int i = 0;
        while (opModeIsActive()) {
            robot.update();

            this.testButton.update(this.gamepad1.b);
            if (this.testButton.isPressed()) {
                telemetry.addData("button", "pressed %d", i++);

                if (RobotConfig.cameraEnabled) {
                    telemetry.addData("Target Strafe offset", robot.vision.getTargetStrafeOffset());
                    telemetry.addData("Target Forward offset", robot.vision.getTargetForwardOffset());
                }
                telemetry.update();
            }

        }
    }
}

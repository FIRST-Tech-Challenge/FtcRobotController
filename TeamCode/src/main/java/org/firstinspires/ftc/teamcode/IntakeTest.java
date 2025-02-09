package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.bots.RollerIntakeBot;

@TeleOp(name = "IntakeTest")
public class IntakeTest extends LinearOpMode {
    private RollerIntakeBot rollerIntakeBot;

    @Override
    public void runOpMode() throws InterruptedException {
        rollerIntakeBot = new RollerIntakeBot(this);
        rollerIntakeBot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            rollerIntakeBot.onLoop(0,"test");
            if(gamepad1.dpad_up){
                rollerIntakeBot.intake();
            }
            if(gamepad1.dpad_down){
                rollerIntakeBot.outake();
            }
            if(gamepad1.dpad_left){
                rollerIntakeBot.stopRoller();
            }
            if (gamepad1.a) {
                rollerIntakeBot.logColorSensor(); // Get colour values
            }
            if (gamepad1.x) {
                rollerIntakeBot.adjustGain(true); // Increase gain
            } else if (gamepad1.y) {
                rollerIntakeBot.adjustGain(false); // Decrease gain
            }

            if (gamepad1.right_bumper) {
//                gamepad1.
//                gamepad1.rumble(5000);
                rollerIntakeBot.setColorIndicator(0.3);
            }
            if(gamepad1.left_bumper){
                int a = rollerIntakeBot.getObjectInPlace();
                telemetry.addData("current", a);
            }
//            gamepad1.rumble
            telemetry.update();
        }
    }
}
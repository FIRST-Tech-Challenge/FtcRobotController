package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.EncoderBot;

@TeleOp(name="EncoderMode", group="Robot15173")
//@Disabled
public class EncoderMode extends LinearOpMode {
    EncoderBot robot = new EncoderBot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        try {
            try {
                robot.init(this, this.hardwareMap, telemetry);
            }
            catch (Exception ex){
                telemetry.addData("Init", ex.getMessage());
            }

            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                // POV Mode uses left stick to go forward, and right stick to turn.
                // - This uses basic math to combine motions and is easier to drive straight.
                double drive = gamepad1.left_stick_y;
                double turn = 0;
                double ltrigger = gamepad1.left_trigger;
                double rtrigger = gamepad1.right_trigger;
                if (ltrigger > 0){
                    turn = -ltrigger;
                }
                else if (rtrigger > 0){
                    turn = rtrigger;
                }
                robot.move(drive, turn);

                telemetry.update();
            }
        }
        catch (Exception ex){
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
        }
    }
}

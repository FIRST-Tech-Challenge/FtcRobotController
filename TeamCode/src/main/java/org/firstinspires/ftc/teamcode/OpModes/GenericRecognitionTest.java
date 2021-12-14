package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.bots.OutreachBot;
import org.firstinspires.ftc.teamcode.skills.GenericDetector;
import org.firstinspires.ftc.teamcode.skills.RingDetector;

// Control Hub ADB Terminal Command for Reference
// adb.exe connect 192.168.43.1:5555

@TeleOp(name = "Rec test Thread", group = "Robot15173")
@Disabled
public class GenericRecognitionTest extends LinearOpMode {

    // Declare OpMode members.
    private GenericDetector rf = null;
    private String result = "";
    private OutreachBot bot = new OutreachBot();

    @Override
    public void runOpMode() {
        try {
            try {
                //initialize the bot
                bot.init(this, this.hardwareMap, telemetry);

                //initialize the detector. It will run on its own thread continuously
                rf = new GenericDetector(this.hardwareMap,  this,  telemetry);
                Thread detectThread = new Thread(rf);
                detectThread.start();
                telemetry.update();
            } catch (Exception ex) {
                telemetry.addData("Error", String.format("Unable to initialize Detector. %s", ex.getMessage()));
                telemetry.update();
                sleep(5000);
                return;
            }

            // Wait for the game to start (driver presses PLAY)
            telemetry.update();
            waitForStart();

            rf.stopDetection();

            result = rf.getResult();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                //move the bot
                double drive = gamepad1.left_stick_y;
                bot.move(drive);

                //show recognition result
                telemetry.addData("Detection result", result);
                telemetry.update();
            }
        } catch (Exception ex) {
            telemetry.addData("Init Error", ex.getMessage());
            telemetry.update();
        } finally {
            if (rf != null) {
                rf.stopDetection();
            }
        }
    }
}

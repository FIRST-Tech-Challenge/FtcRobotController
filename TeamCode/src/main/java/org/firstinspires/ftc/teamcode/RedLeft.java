package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.network.WifiUtil;
import org.firstinspires.ftc.teamcode.hardware.MecanumEncoder;

@Autonomous(name="Red Left")
public class RedLeft extends LinearOpMode {

    private MecanumEncoder drive = new MecanumEncoder(this);
    private String TESTBOT = "24342-RC";
    private String wifiSsid = "";
    private Telemetry.Item debugOutout_tel = null;
    private Telemetry.Item parking_status_tel = null;
    private Telemetry.Item parking_delay_tel = null;
    boolean parking_status = false;
    double parking_delay = 0.0;
    Gamepad previous = new Gamepad();
    Gamepad current = new Gamepad();

    @Override
    public void runOpMode()  throws InterruptedException {
        wifiSsid = WifiUtil.getConnectedSsid();


        // run once when init is pressed
        drive.initHardware(this.hardwareMap, wifiSsid.equals(TESTBOT) ? MecanumEncoder.Bot.TestBot : MecanumEncoder.Bot.CompBot);
        drive.resetYaw();
        telemetry.clearAll();
        telemetry.setAutoClear(false);
        //debugOutout = telemetry.addData("Debug:", "");
        parking_status_tel = telemetry.addData("Parking Status:", "%s", "");
        parking_delay_tel = telemetry.addData("Parking Delay:", "%0.1f", 0.0);
        telemetry.update();

        previous.copy(gamepad1);
        current.copy(gamepad1);
        while (opModeInInit()) {
            current.copy(gamepad1);
            if (current.a && !previous.a) {
                parking_status = !parking_status;
            }

            if (current.dpad_up && !previous.dpad_up) {
                if (parking_delay < 10.0) {
                    parking_delay = parking_delay + 0.5;
                }
            }
            if (current.dpad_down && !previous.dpad_down) {
                if (parking_delay > 0.5) {
                    parking_delay = parking_delay - 0.5;
                }
            }

            parking_status_tel.setValue(parking_status ? "true" : "false");
            parking_delay_tel.setValue(parking_delay);
            telemetry.update();
            previous.copy(current);
        }

        telemetry.update();

        // After we are done initializing our code, we wait for Start button.
        waitForStart();
            // move off the wall
        drive.driveForwardInches(0.4,2,15.0);
            // move to in front of first sample and push it into observation
        drive.driveLeftInches(0.4,27,15.0);
            //push the first piece forward and push the second peice in for a score
        drive.driveRightInches(0.4,8,15.0);
        drive.driveForwardInches(0.7,47,15.0);
        drive.driveLeftInches(0.4,9,15.0);
        drive.driveBackwardInches(0.7,42,15.0);
            // go forward and then right and push the 3nd sample into observation
        drive.driveForwardInches(0.7,42,15.0);
        drive.driveLeftInches(0.4,6.5,15.0);
        drive.driveBackwardInches(0.7,39,15.0);
            // repeat the above^^^^ for third sample
//        drive.driveForwardInches(0.7,42,15.0); // for testbot,  for comp bot
//        drive.driveLeftInches(0.4,8,15.0); // for testbot, for comp bot
//        drive.driveBackwardInches(0.7,42,15.0); // for testbot, for comp bot
        //correct yellow position
        drive.driveRightInches(0.7, 8, 15.0); // for testbot, for comp bot
        drive.driveForwardInches(0.7, 40, 15.0); // for testbot, for comp bot
        drive.rotateCounterClockwise(0.4,22,15.0); // for testbot,  for comp bot

        drive.driveBackwardInches(0.7,30,15.0); // for testbot,  for comp bot
        //drive.driveForwardInches(0.47,4,15.0); // for testbot, for comp bot
        //drive.driveRightInches(0.47,8.5,15.0); // for testbot,  for comp bot
        //drive.driveBackwardInches(0.47,6.5,15.0); // for testbot, for comp bot
        drive.stop();
    }
}

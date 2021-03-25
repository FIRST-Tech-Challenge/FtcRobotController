package org.firstinspires.ftc.teamcode.OpModes;

import android.hardware.camera2.CameraCharacteristics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.bots.DummyBot;
import org.firstinspires.ftc.teamcode.skills.Led;
import org.firstinspires.ftc.teamcode.skills.RingDetector;
import org.firstinspires.ftc.teamcode.tfrec.Detector;
import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier;

import java.util.List;

// Control Hub ADB Terminal Command for Reference
// adb.exe connect 192.168.43.1:5555

@TeleOp(name = "Ring Rec Thread", group = "Robot15173")
//@Disabled
public class RingRecogTest extends LinearOpMode {

    // Declare OpMode members.
    private RingDetector rf = null;
    private AutoDot wobdot = new AutoDot();

    @Override
    public void runOpMode() {
        try {
            try {
//                Led lights = new Led();
//                lights.init(this.hardwareMap, telemetry);
                rf = new RingDetector(this.hardwareMap, AutoRoute.NAME_RED, this, null, telemetry);
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

            wobdot = rf.getRecogZone();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                telemetry.addData("Zone", wobdot.getDotName());
                telemetry.addData("X-Coord", wobdot.getX());
                telemetry.addData("Y-Coord", wobdot.getY());
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

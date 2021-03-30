package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.skills.Led;
import org.firstinspires.ftc.teamcode.skills.RingDetector;

// Control Hub ADB Terminal Command for Reference
// adb.exe connect 192.168.43.1:5555

@TeleOp(name = "Ring Cropped Test", group = "Robot15173")
@Disabled
public class RingRecogCroppedTest extends LinearOpMode {

    // Declare OpMode members.
    private RingDetector rf = null;
    private AutoDot wobdot = new AutoDot();
    private float centercoord = 0;

    @Override
    public void runOpMode() {
        try {
            try {
//                Led lights = new Led();
//                lights.init(this.hardwareMap, telemetry);
                rf = new RingDetector(this.hardwareMap, AutoRoute.NAME_RED, this, null, telemetry, "croppedRingRec.tflite", "croppedLabels.txt");
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
            centercoord = rf.getRingCenter();


            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                telemetry.addData("Zone", wobdot.getDotName());
                telemetry.addData("X-Coord", wobdot.getX());
                telemetry.addData("Y-Coord", wobdot.getY());
                telemetry.addData("Center", centercoord);
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

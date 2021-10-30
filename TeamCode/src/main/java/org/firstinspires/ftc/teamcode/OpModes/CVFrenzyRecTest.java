package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CVRec.CVDetectMode;
import org.firstinspires.ftc.teamcode.CVRec.CVDetector;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;

// Control Hub ADB Terminal Command for Reference
// adb.exe connect 192.168.43.1:5555

@TeleOp(name = "CV Frenzy Rec", group = "Robot15173")
//@Disabled
public class CVFrenzyRecTest extends LinearOpMode {

    CVDetector detector;

    @Override
    public void runOpMode() {
        try {

            detector = new CVDetector(this.hardwareMap, AutoRoute.NAME_RED);
            detector.init(CVDetectMode.Frenzy, "wcam", "cameraMonitorViewId");

            detector.startDetection();
            telemetry.addData("Info", "Detector initialized");
            telemetry.update();

            waitForStart();


            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                telemetry.addData("Element: ", detector.getGameElement());
                telemetry.addData("Mean Val: ", detector.getMeanVal());
                telemetry.update();
                sleep(200);
            }
        } catch (Exception ex) {
            telemetry.addData("Init Error", ex.getMessage());
            telemetry.update();
        } finally {
            if (detector != null) {
                detector.stopDetection();
            }
        }
    }
}

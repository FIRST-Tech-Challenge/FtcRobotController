package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CVRec.CVDetectMode;
import org.firstinspires.ftc.teamcode.CVRec.CVDetector;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.skills.Led;
import org.firstinspires.ftc.teamcode.skills.RingDetector;

// Control Hub ADB Terminal Command for Reference
// adb.exe connect 192.168.43.1:5555

@TeleOp(name = "CV Ring Rec", group = "Robot15173")
//@Disabled
public class CVRingRecogTest extends LinearOpMode {

    CVDetector detector;

    @Override
    public void runOpMode() {
        try {

            detector = new CVDetector(this.hardwareMap);
            detector.init(CVDetectMode.Stack, "wcam", "cameraMonitorViewId");

            detector.startDetection();
            telemetry.addData("Info", "Detector initialized");
            telemetry.update();

            waitForStart();


            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                telemetry.addData("Stack size", detector.getStackSize());
                telemetry.addData("Mean Val", detector.getMeanVal());
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

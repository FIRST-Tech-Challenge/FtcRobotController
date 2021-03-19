package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.CVRec.CVDetectMode;
import org.firstinspires.ftc.teamcode.CVRec.CVDetector;
import org.firstinspires.ftc.teamcode.CVRec.CVRoi;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

// Control Hub ADB Terminal Command for Reference
// adb.exe connect 192.168.43.1:5555

@TeleOp(name = "CV Ring Search", group = "Robot15173")
//@Disabled
public class CVRingSearchTest extends LinearOpMode {

    CVDetector detector;
    Deadline gamepadRateLimit;
    private final static int GAMEPAD_LOCKOUT = 500;

    @Override
    public void runOpMode() {
        try {

            gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

            detector = new CVDetector(this.hardwareMap);
            detector.init(CVDetectMode.Search, "wcam", "cameraMonitorViewId");

            detector.startDetection();
            telemetry.addData("Info", "Detector initialized");
            telemetry.update();

            waitForStart();


            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                if (gamepad1.x){
                    gamepadRateLimit.reset();
                    List<CVRoi> list = detector.getTargets();
                    if (list != null && list.size() > 0){
                        telemetry.addData("Number of targets", list.size());
                        for(int i = 0; i < list.size(); i++) {
                            CVRoi target = list.get(i);
                            telemetry.addData(String.format("%.2f, Angle", i), target.getMeanVal());
                            telemetry.addData(String.format("%d, Angle", i), target.getAngle());
                            telemetry.addData(String.format("%d, Clockwise", i), target.isClockwise());
                            telemetry.addData(String.format("%d, Distance", i), target.getDistance());
                        }
                        telemetry.update();
                    }
                }
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

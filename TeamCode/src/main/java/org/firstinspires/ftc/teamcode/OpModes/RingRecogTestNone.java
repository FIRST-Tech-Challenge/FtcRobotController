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
// /sdcard/FIRST/matchlogs/Match-0-Red-3.txt

@TeleOp(name = "Ring Rec", group = "Robot15173")
//@Disabled
public class RingRecogTestNone extends LinearOpMode {

    // Declare OpMode members.
    private RingDetector rf = null;
    private AutoDot wobdot = new AutoDot();

    @Override
    public void runOpMode() {
        try {
            try {
                Led lights = new Led();
                lights.init(this.hardwareMap, telemetry);
                rf = new RingDetector(this.hardwareMap, AutoRoute.NAME_RED, this, null, lights, telemetry);
                telemetry.addData("Info", "Opmode initialized");
                telemetry.update();
            } catch (Exception ex) {
                telemetry.addData("Error", String.format("Unable to initialize Detector. %s", ex.getMessage()));
                telemetry.update();
            }

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            wobdot = rf.detectRing(2, AutoRoute.NAME_RED, telemetry, this);

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

package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.CVRec.CVDetectMode;
import org.firstinspires.ftc.teamcode.CVRec.CVDetector;
import org.firstinspires.ftc.teamcode.CVRec.CVRoi;
import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;
import org.firstinspires.ftc.teamcode.bots.UltimateBot;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;

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
    RobotCoordinatePosition locator = null;
    UltimateBot robot   = new UltimateBot();
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        try {

            gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

            detector = new CVDetector(this.hardwareMap);
            detector.init(CVDetectMode.Search, "wcam", "cameraMonitorViewId");

            detector.startDetection();
            telemetry.addData("Info", "Detector initialized");
            telemetry.update();

            try {
                robot.init(this, this.hardwareMap, telemetry);
                robot.initGyro();
                robot.initCalibData();
            }
            catch (Exception ex){
                telemetry.addData("Init", ex.getMessage());
            }

            locator = new RobotCoordinatePosition(robot, new Point(30, 24), 0,RobotCoordinatePosition.THREAD_INTERVAL);
            locator.reverseHorEncoder();
            Thread positionThread = new Thread(locator);
            positionThread.start();

            waitForStart();


            double nextHead = 0;
            while (opModeIsActive()) {

                CVRoi closest = detector.getNearestTarget();;
                CVRoi next = detector.getSecondTarget();

                if (closest != null) {
                    telemetry.addData("Nearest Index", closest.getIndex());
                    telemetry.addData("Nearest Val", closest.getMeanVal());
                    telemetry.addData("Nearest Angle", closest.getAngle());
                    telemetry.addData("Nearest Clockwise", closest.isClockwise());
                    telemetry.addData("Nearest Distance", closest.getDistance());
                    telemetry.addData("Nearest Is Merged", closest.isMerged());
                    double curHead = locator.getAdjustedCurrentHeading();
                    if (closest.isClockwise()) {
                        nextHead = curHead + closest.getAngle();
                        if (nextHead > 360) {
                            nextHead = nextHead - 360;
                        }
                    } else {
                        nextHead = curHead - closest.getAngle();
                        if (nextHead < 0) {
                            nextHead = 360 + nextHead;
                        }
                    }
                    telemetry.addData("Robot", String.format("Turn to %.2f degrees", nextHead));
                }

                if (next != null){
                    telemetry.addData("Next Index", next.getIndex());
                    telemetry.addData("Next Val", next.getMeanVal());
                    telemetry.addData("Next Angle", next.getAngle());
                    telemetry.addData("Next Clockwise", next.isClockwise());
                    telemetry.addData("Next Distance", next.getDistance());
                }

                if (gamepad1.x){
                    gamepadRateLimit.reset();
                    BotMoveProfile profileSpin = BotMoveProfile.getFinalHeadProfile(nextHead, 0.3, locator);
                    robot.spin(profileSpin, locator);

                    timer.reset();
                    while (timer.milliseconds() < 1000){
                        //do nothing
                    }
                    nextHead = locator.getAdjustedCurrentHeading();
                }

                telemetry.update();
            }
        } catch (Exception ex) {
            telemetry.addData("Init Error", ex.getMessage());
            telemetry.update();
        } finally {
            if (detector != null) {
                detector.stopDetection();
            }
            if (locator != null){
                locator.stop();
            }
        }
    }
}

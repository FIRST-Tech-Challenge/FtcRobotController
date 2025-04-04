package org.firstinspires.ftc.team13590.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.team13590.RobotHardware;
import org.firstinspires.ftc.team13590.VisionSoftware;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

@TeleOp(name = "Vision: test", group = "Robot")
public class VisionTest extends LinearOpMode {
    @SuppressLint("DefaultLocale")

    RobotHardware robot = new RobotHardware(this);
    VisionSoftware.colorDetector colorDetector = new VisionSoftware.colorDetector(this);
    ElapsedTime runtime = new ElapsedTime();

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode()
    {
        RotatedRect boxFit;
        double relevantAngle;
        YawPitchRollAngles  yawAngles;

        robot.init(false);
        colorDetector.visionInit("BLUE", false, -0.5, 0.5, 0.5, -0.5);
        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        colorDetector.portalColor.setProcessorEnabled(colorDetector.primaryColorProcessor, true);
        while (opModeInInit())
        {
            yawAngles = robot.imu.getRobotYawPitchRollAngles(); // set orientation
            telemetry.addData("check preview, initialized", "... Camera Stream");
            telemetry.addData("current orientation", String.valueOf(yawAngles));
            colorDetector.activeDetector(new Point(0,0), new Point(10000,10000), "PRIMARY"); // run camera

        }

        waitForStart();
        robot.imu.resetYaw(); // reset orientation
        runtime.reset();

        while (opModeIsActive()) { // once in play, print orientation

            /* the plan is to check orientation and draw a master north line in camera, thus giving it a sense
                of direction when thrown of course. This will also allow more precise camera-based movement.
                The line should move along with the camera: if the robot rotates 30deg ccw, the line will move
                30deg to the right projected(how it will display in 2d) onto the camera display.
            */
            colorDetector.activeDetector(new Point(0,0), new Point(10000,10000), "PRIMARY"); // run camera

            if(!colorDetector.primaryBlobList.isEmpty() ) {
                // find the RotatedRect class of the largest blob
                boxFit = colorDetector.primaryBlobList.get(0).getBoxFit();
                // save that blob's angle
                relevantAngle = Math.round(boxFit.angle);
                // assume claw straight ahead is 90 deg; cw is + deg, ccw is - deg; maximum is 300 total deg of freedom
                // assume 0.0 is ccw and -60 deg, 1.0 is 240 deg cw
                // slope for angle-to-position
                robot.clawYaw.setPosition(0.0033333*relevantAngle+0.2);
            }
            telemetry.update();
            sleep(50);
        }

    }
}

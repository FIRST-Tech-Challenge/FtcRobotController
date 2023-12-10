package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.SmartGamepad;
import org.firstinspires.ftc.teamcode.subsystems.RobotVision;
import org.firstinspires.ftc.teamcode.subsystems.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.Utilities;

@Config
@TeleOp
public class CameraTest extends LinearOpMode {
    public static boolean IS_RED = false;
    public static boolean ALIGN_RIGHT = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Utilities.getSharedUtility().initialize(this);
        waitForStart();
        RobotVision rvis = new RobotVision(ALIGN_RIGHT);

        NanoClock clock = NanoClock.system();
        double prevTime = clock.seconds();

        int elementPos = 3;

        while (!isStopRequested()) {
            telemetry.update();
            elementPos = rvis.getTeamPropOrientation(IS_RED, ALIGN_RIGHT);
            telemetry.addData("team prop pos: ", elementPos);
            //Log.v("arm", "right servo position: "+ robot.outtake.getRightServoPos());
            double currentTime = clock.seconds();
            telemetry.addData("Update time: ", currentTime - prevTime);
            prevTime = currentTime;
        }
    }
}

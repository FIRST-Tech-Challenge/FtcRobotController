package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.subsystems.RobotVision;
import org.firstinspires.ftc.teamcode.util.Utilities;
@Config
@TeleOp
public class TouchSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Utilities.getSharedUtility().initialize(this);
        TouchSensor sensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        waitForStart();

        NanoClock clock = NanoClock.system();
        double prevTime = clock.seconds();

        int elementPos = 3;

        while (!isStopRequested()) {
            telemetry.update();
            telemetry.addData("touch sensor", sensor.isPressed());
            //Log.v("arm", "right servo position: "+ robot.outtake.getRightServoPos());
            double currentTime = clock.seconds();
            telemetry.addData("Update time: ", currentTime - prevTime);
            prevTime = currentTime;
        }
    }
}

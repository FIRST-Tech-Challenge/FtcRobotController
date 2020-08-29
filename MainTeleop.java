package org.firstinspires.ftc.teamcode.rework;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.rework.ModuleTools.TelemetryProvider;
import org.firstinspires.ftc.teamcode.rework.RobotTools.TelemetryDump;

import java.util.HashMap;
import java.util.Map;

@TeleOp
public class MainTeleop extends LinearOpMode implements TelemetryProvider {
    Robot robot;
    long lastUpdateTime;

    public void runOpMode() {
        TelemetryDump.registerProvider(this);
        initRobot();
        waitForStart();
        robot.startModules();

        while (opModeIsActive()) {
            updateDrivetrainStates();
            lastUpdateTime = SystemClock.elapsedRealtime();
        }
    }

    private void initRobot() {
        robot = new Robot(hardwareMap, telemetry,this);
        robot.initModules();
    }

    private void updateDrivetrainStates() {
        robot.drivetrainModule.yMovement = -gamepad1.left_stick_y;
        robot.drivetrainModule.xMovement = gamepad1.left_stick_x;
        robot.drivetrainModule.turnMovement = gamepad1.right_stick_x;
    }

    @Override
    public Map<String, String> getTelemetryData() {
        long currentTime = SystemClock.elapsedRealtime();

        HashMap<String, String> data = new HashMap<>();
        data.put("---TeleOp Update Speed---", "");
        data.put("TeleOp while loop update time: ", String.valueOf(currentTime - lastUpdateTime));
        lastUpdateTime = currentTime;

        return data;
    }
}
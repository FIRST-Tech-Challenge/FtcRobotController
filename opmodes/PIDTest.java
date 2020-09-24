package org.firstinspires.ftc.teamcode.rework.opmodes;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.rework.util.auto.PIDController;
import org.firstinspires.ftc.teamcode.rework.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.rework.Robot;

import java.util.ArrayList;

@TeleOp
public class PIDTest extends LinearOpMode implements TelemetryProvider {
    Robot robot;
    long lastUpdateTime;

    public void runOpMode() {
        robot.telemetryDump.registerProvider(this);
        initRobot();
        PIDController pidController = new PIDController(0.01,0.0000001,0,robot);
        waitForStart();
        robot.startModules();

        while (opModeIsActive()) {
            long initialTime = SystemClock.elapsedRealtime();
            pidController.PID(robot.odometryModule.worldY,56);
            robot.drivetrainModule.yMovement = pidController.scale;
            lastUpdateTime = SystemClock.elapsedRealtime() - initialTime;
        }
    }

    private void initRobot() {
        robot = new Robot(hardwareMap, telemetry,this);
        robot.initModules();
    }

    private void updateDrivetrainStates() {
        robot.drivetrainModule.yMovement = gamepad1.left_stick_y;
        robot.drivetrainModule.xMovement = gamepad1.left_stick_x;
        robot.drivetrainModule.turnMovement = gamepad1.right_stick_x;
    }


    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Module Executor thread loop time: " + String.valueOf(lastUpdateTime));
        return data;
    }

    public String getName() {
        return "PIDTest";
    }
}


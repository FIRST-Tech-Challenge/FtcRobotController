package org.firstinspires.ftc.teamcode.rework;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.rework.ModuleTools.TelemetryProvider;
import org.firstinspires.ftc.teamcode.rework.RobotTools.TelemetryDump;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

@TeleOp
public class MainTeleop extends LinearOpMode implements TelemetryProvider {
    Robot robot;
    long lastUpdateTime;

    private double SLOW_MODE_SCALE_FACTOR = 0.3;

    private boolean lastGlobalMoveState = false;
    private double globalMoveAngle = 0;

    public void runOpMode() {
        initRobot();
        robot.telemetryDump.registerProvider(this);
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

        double yMovement = 0;
        double xMovement = 0;
        double turnMovement = 0;

        if (usingJoysticks()){
            yMovement = -gamepad1.left_stick_y;
            xMovement = gamepad1.left_stick_x;
            turnMovement = gamepad1.right_stick_x;
        } else if(usingDPad()){
            if (gamepad1.dpad_up){
                yMovement = 1;
            } else if (gamepad1.dpad_down){
                yMovement = -1;
            } else if (gamepad1.dpad_left){
                turnMovement = -1;
            } else if (gamepad1.dpad_right){
                turnMovement = 1;
            }
        }

        if (usingGlobalMove()){
            if (!lastGlobalMoveState){
                globalMoveAngle = robot.odometryModule.worldAngleRad;
                lastGlobalMoveState = true;
            }

            double r = Math.hypot(yMovement, xMovement);
            double aT = Math.atan2(yMovement, xMovement);
            double t = aT + robot.odometryModule.worldAngleRad - globalMoveAngle;

            double nXMovement = r * Math.cos(t);
            double nYMovement = r * Math.sin(t);

            xMovement = nXMovement;
            yMovement = nYMovement;
        } else {
            lastGlobalMoveState = false;
        }

        if(usingSlowMode()){
            xMovement *= SLOW_MODE_SCALE_FACTOR;
            yMovement *= SLOW_MODE_SCALE_FACTOR;
            turnMovement *= SLOW_MODE_SCALE_FACTOR;
        }

        robot.drivetrainModule.yMovement = yMovement;
        robot.drivetrainModule.xMovement = xMovement;
        robot.drivetrainModule.turnMovement = turnMovement;

    }

    private boolean usingJoysticks(){
        return gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0;
    }

    private boolean usingDPad(){
        return gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;
    }

    private boolean usingGlobalMove(){
        return gamepad1.left_bumper;
    }

    private boolean usingSlowMode(){
        return gamepad1.right_bumper;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        long currentTime = SystemClock.elapsedRealtime();

        ArrayList<String> data = new ArrayList<>();
        data.add("TeleOp while loop update time: " + String.valueOf(currentTime - lastUpdateTime));
        lastUpdateTime = currentTime;

        return data;
    }
}
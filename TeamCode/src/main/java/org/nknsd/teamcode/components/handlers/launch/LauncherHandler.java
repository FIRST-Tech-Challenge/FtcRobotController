package org.nknsd.teamcode.components.handlers.launch;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNComponent;

public class LauncherHandler implements NKNComponent {

    private DcMotorEx wMotor;
    private boolean enabled = true;
    private final double lowerThreshold;
    private final double upperThreshold;
    private double targetTps;
    private int confidenceNum;
    private double previousRunTime;
    private final double TICK_COUNT_INTERVAL_SECONDS = 0.05;


    public LauncherHandler(double lowerThreshold, double upperThreshold) {
        this.lowerThreshold = lowerThreshold;
        this.upperThreshold = upperThreshold;
    }

    public void setTargetTps(double targetTps) {
        this.targetTps = targetTps;
    }


    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        wMotor = (DcMotorEx) hardwareMap.dcMotor.get("LM");
        wMotor.setVelocityPIDFCoefficients(100,2,2,0);
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {
    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {
    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "LauncherHandler";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        if (!enabled) {
            wMotor.setPower(0);
            //wPower = 0;
            return;
        }

        wMotor.setVelocity(targetTps);
//
        double timeElapsed = runtime.seconds() - previousRunTime;
        if (timeElapsed < TICK_COUNT_INTERVAL_SECONDS) {
            return;
        }
//        int wMotorPosition = wMotor.getCurrentPosition();
//        RobotLog.v("motor position " + wMotorPosition);
//        ticks = Math.abs(wMotorPosition - wMotorPositionPrevious);
//        RobotLog.v("ticks " + ticks);
//        currentTps = ticks / timeElapsed;
//        RobotLog.v("current tps " + currentTps);

        if (wMotor.getVelocity() > targetTps * lowerThreshold && wMotor.getVelocity() < targetTps * upperThreshold) {
            confidenceNum++;
        } else {
            confidenceNum = 0;
        }
//        if (!powerLocked) {
//            wPower = wPower + (pFactor * (targetTps - currentTps));
//        }
//        wMotorPositionPrevious = wMotorPosition;
        previousRunTime = runtime.seconds();
//        if (wPower < 0) {
//            wPower = 0;
//        }
//        if (wPower > 1) {
//            wPower = 1;
//        }
//        if (targetTps * upperThreshold  * upperThreshold < currentTps) {
//            wMotor.setPower(0);
//        } else if (targetTps * lowerThreshold * lowerThreshold  > currentTps) {
//            wMotor.setPower(1);
//        } else {
//            wMotor.setPower(wPower);
//        }
    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("currentTps", wMotor.getVelocity());
        telemetry.addData("vel", wMotor.getVelocity());
        telemetry.addData("targetTps", targetTps);
        telemetry.addData("confidence", confidenceNum);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public double getCurrentTps() {
        return wMotor.getVelocity();
    }


//    how many times in a row the wheel has been good to launch
    public int launchConfidence() {
        return confidenceNum;
    }

    public void resetConfidence() {
        confidenceNum = 0;
    }

}

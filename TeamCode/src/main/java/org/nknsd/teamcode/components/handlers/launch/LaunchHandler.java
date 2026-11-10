package org.nknsd.teamcode.components.handlers.launch;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNComponent;

/// THIS CLASS IS NOT READY YET
/// PLEASE DO NOT USE IT UNLESS
/// THIS TEXT IS GONE
/// CHECK THE SOFTWARE ARCHITECTURE TO SEE WHAT IT DOES
public class LaunchHandler implements NKNComponent {

    private DcMotorEx leftLaunch;
    private DcMotorEx rightLaunch;
    private boolean enabled = true;
    private final double lowerThreshold;
    private final double upperThreshold;
    private double targetTps;
    private int confidenceNum;
    private double previousRunTime;
    private final double sampleInterval = 0.05;


    public LaunchHandler(double lowerThreshold, double upperThreshold) {
        this.lowerThreshold = lowerThreshold;
        this.upperThreshold = upperThreshold;
    }

    public void setTargetTicks(double targetTps) {
        this.targetTps = targetTps;
    }


    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        leftLaunch = (DcMotorEx) hardwareMap.dcMotor.get("leftLaunch");
        rightLaunch = (DcMotorEx) hardwareMap.dcMotor.get("rightLaunch");
        rightLaunch.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLaunch.setVelocityPIDFCoefficients(100,2,2,0);
        rightLaunch.setVelocityPIDFCoefficients(100,2,2,0);
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
            leftLaunch.setPower(0);
            rightLaunch.setPower(0);
            //wPower = 0;
            return;
        }

        leftLaunch.setVelocity(targetTps);
        rightLaunch.setVelocity(targetTps);
//
        double timeElapsed = runtime.seconds() - previousRunTime;
        if (timeElapsed < sampleInterval) {
            return;
        }
//        int wMotorPosition = wMotor.getCurrentPosition();
//        RobotLog.v("motor position " + wMotorPosition);
//        ticks = Math.abs(wMotorPosition - wMotorPositionPrevious);
//        RobotLog.v("ticks " + ticks);
//        currentTps = ticks / timeElapsed;
//        RobotLog.v("current tps " + currentTps);

        if ((leftLaunch.getVelocity() > targetTps * lowerThreshold && leftLaunch.getVelocity() < targetTps * upperThreshold) &&
            (rightLaunch.getVelocity() > targetTps * lowerThreshold && rightLaunch.getVelocity() < targetTps * upperThreshold)) {
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
        telemetry.addData("leftTps", leftLaunch.getVelocity());
        telemetry.addData("rightTps", rightLaunch.getVelocity());
        telemetry.addData("targetTps", targetTps);
        telemetry.addData("confidence", confidenceNum);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
    public boolean isEnabled(){
        return enabled;
    }

//    public double getLeftTps() {
//        return leftLaunch.getVelocity();
//    }
//    public double getRightTps() {
//        return rightLaunch.getVelocity();
//    }


    //    how many times in a row the wheel has been good to launch
    public int getConfidence() {
        return confidenceNum;
    }

    public void resetConfidence() {
        confidenceNum = 0;
    }

}

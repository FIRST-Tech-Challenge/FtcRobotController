package org.nknsd.teamcode.components.handlers.launch;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.google.blocks.ftcrobotcontroller.hardware.HardwareItemMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.components.utility.feedbackcontroller.PidController;
import org.nknsd.teamcode.frameworks.NKNComponent;

public class LaunchHandler implements NKNComponent {

    //motors
    DcMotorEx leftLaunch;
    DcMotorEx rightLaunch;
    String leftLaunchName = "launchLeft";
    String rightLaunchName = "rightLaunch";

    //servos
    CRServo transferLeft;
    CRServo transferRight;
    String leftTransferName = "leftTrans";
    String rightTransferName = "rightTrans";

    //constants
    private final double servoSpeed = 1;

    //pid values are arbitrary and can freely be adjusted
    PidController pidController = new PidController(0.001, 1, 0.5,0.3,false, 1000, 0.5);

    //important variables
    boolean launchEnabled = false;

    double targetVelocity = 3000;
    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        leftLaunch = (DcMotorEx) hardwareMap.get(leftLaunchName);
        rightLaunch = (DcMotorEx) hardwareMap.get(rightLaunchName);
        transferLeft = hardwareMap.crservo.get(leftTransferName);
        transferRight = hardwareMap.crservo.get(rightTransferName);
        transferRight.setDirection(DcMotorSimple.Direction.REVERSE);
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
        return "";
    }

    double leftErr;
    double leftErrDelta;
    double rightErr;
    double rightErrDelta;
    double loopStart;
    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        if(launchEnabled){
            double leftVel = ;
            double rightVel = ;
            setLauncherPower(leftErr, leftErrDelta);
        } else {
            setLauncherPower(0);
        }
    }

    @Override
    public void doTelemetry(Telemetry telemetry) {

    }

    public void setTargetVelocity(double targetVelocity){
        this.targetVelocity = targetVelocity;
    }
    public void setLaunchEnabled(boolean launching){
        launchEnabled = launching;

    }

    public void setTransfer(boolean activateTransfer){
        if(activateTransfer) {
            transferLeft.setPower(servoSpeed);
            transferRight.setPower(servoSpeed);
        } else {
            transferLeft.setPower(0);
            transferRight.setPower(0);
        }
    }
    public double getTargetVelocity(){
        return targetVelocity;
    }
    public boolean getLaunching(){
        return launchEnabled;
    }
    private void setLauncherPower(double leftErr, double leftErrDelta, double leftVel, double rightErr, double rightErrDelta, double rightVel, double interval){
        leftLaunch.setPower(pidController.findOutput(leftErr,leftErrDelta,leftVel,interval));
        rightLaunch.setPower(pidController.findOutput(rightErr,rightErrDelta,rightVel,interval));
    }

    /**
     * this is the stupid version of setLauncherPower
     * i only use it for setting motor values to 0
     * @param value motor value to set the launchers to
     */
    private void setLauncherPower(double value){
        leftLaunch.setPower(value);
        rightLaunch.setPower(value);
    }
}

package org.nknsd.teamcode.components.handlers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNComponent;

///THIS CLASS IS OUTDATED. ONCE LaunchHandler AND BallSystem
///ARE DONE, IT SHOULD BE REMOVED INSTANTLY PLEASE!!!!
public class BallLiftHandler implements NKNComponent {

    private final String liftServoName = "liftBallServo";
    private final String gateServoName = "liftGateServo";
    private final String leftMotorWheel = "launchLeft";
    private final String rightMotorWheel = "launchRight";

    //servos
    public Servo liftServo;
    public Servo gateServo;

    //motors
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    double motorPower = 0.37;

    //lift servo values
    double liftRaise = 0.18;
    double liftLower = 1;
    double gateOpen = 0.6;
    double gateClosed = 0;


    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        liftServo = hardwareMap.servo.get(liftServoName);
        gateServo = hardwareMap.servo.get(gateServoName);
        leftMotor = hardwareMap.dcMotor.get(leftMotorWheel);
        rightMotor = hardwareMap.dcMotor.get(rightMotorWheel);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {
        liftServo.setPosition(liftLower);
        gateServo.setPosition(gateOpen);
        leftMotor.setPower(motorPower);
        rightMotor.setPower(motorPower);
    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("current liftMax", liftRaise);
        telemetry.addData("current liftMin", liftLower);
        telemetry.addData("current gateMax", gateOpen);
        telemetry.addData("current gateMin", gateClosed);
    }


    public void raiseLift() {
        liftServo.setPosition(liftRaise);
    }
    public void lowerLift() {
        liftServo.setPosition(liftLower);
    }
    public void lowerGate(){
        gateServo.setPosition(gateClosed);
    }
    public void raiseGate(){
        gateServo.setPosition(gateOpen);
    }
    //code to test the servo positon, unecessary but left in case we truly NEED it
//    public void liftPosInc(){
//        liftRaise = liftRaise + 0.02;
//        liftServo.setPosition(liftRaise);
//    }
//    public void liftPosDec(){
//        liftRaise = liftRaise - 0.02;
//        liftServo.setPosition(liftRaise);
//    }
//    public void botPosInc(){
//        liftMin = liftMin + 0.02;
//        liftServo.setPosition(liftMin);
//    }
//    public void botPosDec(){
//        liftMin = liftMin - 0.02;
//        liftServo.setPosition(liftMin);
//    }
}

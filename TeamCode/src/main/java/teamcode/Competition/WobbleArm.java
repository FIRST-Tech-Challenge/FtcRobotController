package teamcode.Competition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.Debug;

public class WobbleArm {
    // is attached to the back of the robot as a rigid arm that rotates at the base to lift
    private static final double OPEN_CLAW = 0.5;
    private static final double CLOSE_CLAW= 1;

    Servo claw;
    DcMotor armMotor, motorEncoder;

    public WobbleArm(HardwareMap hardwareMap, boolean closeClaw) {
        claw = hardwareMap.servo.get("Claw");
        armMotor = hardwareMap.dcMotor.get("Arm");
        motorEncoder = hardwareMap.dcMotor.get("LowerIntakeMotor");
        if (closeClaw) {
            claw.setPosition(CLOSE_CLAW);
        }else{
            claw.setPosition(OPEN_CLAW);
        }
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Debug.log(armMotor.getCurrentPosition());
    }

    public void runToPosition(int ticks, double power){
        motorEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEncoder.setTargetPosition(ticks);
        motorEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Debug.log(!isNearPosition());
        while(!isNearPosition()) {
            armMotor.setPower( -getSign(motorEncoder.getCurrentPosition() - motorEncoder.getTargetPosition())* power);
        }
        armMotor.setPower(0);

    }

    public double getSign(int num){
        if(num < 0){
            return -1;
        }else{
            return 1;
        }
    }

    private boolean isNearPosition() {
        return Math.abs(motorEncoder.getCurrentPosition() - motorEncoder.getTargetPosition()) < 100;
    }

    public void adjustClaw(){
        if(claw.getPosition() == OPEN_CLAW){
            claw.setPosition(CLOSE_CLAW);
        }else if(claw.getPosition() == CLOSE_CLAW){
            claw.setPosition(OPEN_CLAW);
        }else {
            claw.setPosition(OPEN_CLAW);
        }
    }

    public int getCurrentPos(){
        return motorEncoder.getCurrentPosition();
    }

    public void setPower(double power) {
        armMotor.setPower(power);
    }

    public void runToRelativePosition(int ticks, double power) {
        motorEncoder.setTargetPosition(ticks);
        motorEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Debug.log(armMotor.getTargetPosition());
        while(!isNearPosition()) {
            armMotor.setPower( -getSign(motorEncoder.getCurrentPosition() - motorEncoder.getTargetPosition())* power);
            Debug.log(motorEncoder.getCurrentPosition());
        }
        armMotor.setPower(0);
    }

    public void resetEncoder() {
        motorEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

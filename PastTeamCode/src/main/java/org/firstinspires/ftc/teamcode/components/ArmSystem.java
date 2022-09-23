/***
 * Vinay's vision of the elevator arm system:
 * We will have an elevating arm system that will be tilted a bit downward (roughly 30 degrees)
 * The three levels defined are for different heights that the block needs to be released to
 * – we will use a motor to use the arm up/down
 * Once the intake delivers the block to the top of the arm, the block will drop to the end of the arm by gravity,
 * and there there is a stopper that can be lowered using a servo to release the block.
 ***/

package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/***
 *  one motor - to change elevation
 *  POTENTIAL second motor - to change distance/length arm is reaching out
 *  one Servo - to release stopper.
 */

public class ArmSystem {
    // TODO

    public static AnalogInput sensorAsAnalogInput0;
    private final DcMotor elevatorMotor;
    private boolean bool = false;
//    private final Servo releaser;

    public static final double LEVEL_TOP = 0;
    public static final double LEVEL_CAROUSEL = 0.49 - (0.65 - 0.441);//1.1091 - 0.441; // 0.124
    public static final double LEVEL_BOTTOM = 0.967 - (0.65 - 0.441);
    public static final double LEVEL_INTAKE = 1.057; // 1.786 - 0.441 - (0.65 - 0.441);

    // use potentiamotor to detect voltage, then do from there is brian's suggestion

    public void stop() {
        elevatorMotor.setPower(0.0);
    }

    public DcMotor getElevatorMotor() {
        return elevatorMotor;
    }

    public void initMotors() {
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setPower(0);

    }

    public ArmSystem(DcMotor elevatorMotor, AnalogInput sensorAsAnalogInput0){
        this.elevatorMotor = elevatorMotor;
        this.sensorAsAnalogInput0 = sensorAsAnalogInput0;
        this.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    //high - 0.65
    //carousel - 1.25
    // bottom - 1.786
    //lolo - 2.65

    public boolean notTooHigh(){
        return (sensorAsAnalogInput0.getVoltage() > LEVEL_TOP);
    }

    public boolean notTooLow(){
        return (sensorAsAnalogInput0.getVoltage() < LEVEL_INTAKE);
    }

    public double getSensorAsAnalogInput0() {
        return sensorAsAnalogInput0.getVoltage();
    }

    /**
     * Moves arm up
     */

    /**
     * Intakes rings
     */
    public void moveUp() {
        elevatorMotor.setTargetPosition(elevatorMotor.getCurrentPosition() + 200);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotor.setPower(1.0);
    }

    /**
     * Intakes rings
     */
    public void moveDown() {
        elevatorMotor.setTargetPosition(elevatorMotor.getCurrentPosition() - 80);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotor.setPower(-1.0);

    }
    public void move_up() { // TODO
        if (notTooHigh()){
            elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevatorMotor.setPower(0.55);
        }
    }

    /**
     * Moves arm down
     */
    public void move_down() { // TODO
        if (notTooLow()){
            elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevatorMotor.setPower(-0.2);
        }
    }

    public void moveToPosition(double voltage){
        if (getSensorAsAnalogInput0() < voltage) {
            while (getSensorAsAnalogInput0() < voltage) {
                move_down();
            }
        } else if (getSensorAsAnalogInput0() > voltage) {
            while (getSensorAsAnalogInput0() > voltage) {
                move_up();
            }
        }
        stop();
    }

    public boolean moveToMaxHeight(){
        elevatorMotor.setPower(0.75);
        return getSensorAsAnalogInput0() <= ArmSystem.LEVEL_TOP;
    }

    public boolean moveToIntake(){
        elevatorMotor.setPower(-0.75);
        return getSensorAsAnalogInput0() >= ArmSystem.LEVEL_TOP;
    }

    public boolean moveToCarousel(boolean directionNeedsToGo){
        if (directionNeedsToGo){
            elevatorMotor.setPower(0.75);
            return getSensorAsAnalogInput0() <= ArmSystem.LEVEL_CAROUSEL;
        }
        else{
            elevatorMotor.setPower(-0.75);
            return getSensorAsAnalogInput0() >= ArmSystem.LEVEL_CAROUSEL;
        }
    }

    public boolean moveToBottom(boolean directionNeedsToGo){
        if (directionNeedsToGo){
            elevatorMotor.setPower(0.75);
            return getSensorAsAnalogInput0() <= ArmSystem.LEVEL_BOTTOM;
        }
        else{
            elevatorMotor.setPower(-0.75);
            return getSensorAsAnalogInput0() >= ArmSystem.LEVEL_BOTTOM;
        }
    }

    public void moveToPositionTeleOp(double voltage){
        if (getSensorAsAnalogInput0() < voltage) {
            while (getSensorAsAnalogInput0() < voltage) {
                move_down();
            }
        } if (getSensorAsAnalogInput0() > voltage) {
            while (getSensorAsAnalogInput0() > voltage) {
                move_up();
            }
        }
        stop();
    }
}
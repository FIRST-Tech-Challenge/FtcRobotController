package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;


public class Elevator {

    // Elevator Constants
    final double HOME_POWER = -0.1;

    final int ELEVATOR_GROUND = 320;
    final int ELEVATOR_LOW = 520;
    final int ELEVATOR_MID = 720;
    final int ELEVATOR_HIGH = 920;
    final int ELEVATOR_HOME = 50;

    final int    DEAD_BAND = 5;
    final double FAST_LIFT = 0.5;
    final double SLOW_LIFT = 0.2;
    final double SLOW_LOWER = 0;
    final double FAST_LOWER = -0.05;
    final double HOLD_POWER = 0.02;

    final double ENCODER_TO_ANGLEM = 0.0992;
    final double ENCODER_TO_ANGLEC = -37.2;

    // Wrist & Hand Constants
    final double WRIST_HOME_POSITION = 0.6;
    final double HAND_HOME_POSITION = 0.8;
    final double HAND_OPEN = 0.7;
    final double HAND_CLOSE = 1;

    final double WRIST_TO_SERVOM = 0.00374;
    final double WRIST_TO_SERVOC = 0.472;

    // class members (devices)
    private DcMotorEx   liftMaster;
    private DcMotorEx   liftSlave;
    private List<DcMotorEx> motors;
    private Servo       wrist; //smaller values tilt the wrist down
    private Servo       hand;//smaller values open the hand more

    // class members (objects)
    private LinearOpMode myOpMode = null;
    private int     target = 0;
    private int     lastPosition = 0;
    private double  power    = 0;
    private boolean liftActive = false;
    private double  wristOffset = 0;
    private int     currentPosition = 0;

    // Elevator Constructor.  Call once opmode is running.
    public Elevator(LinearOpMode opMode) {
        // Attach to hardware devices
        myOpMode = opMode;
        liftMaster = myOpMode.hardwareMap.get(DcMotorEx.class, "lift_master");
        liftSlave = myOpMode.hardwareMap.get(DcMotorEx.class, "lefte");
        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");
        hand = myOpMode.hardwareMap.get(Servo.class, "hand");
        motors = Arrays.asList(liftMaster, liftSlave);

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMaster.setDirection(DcMotorSimple.Direction.FORWARD);
        liftSlave.setDirection(DcMotorSimple.Direction.REVERSE);
        setWristPosition(WRIST_HOME_POSITION);
        setHandPosition(HAND_HOME_POSITION);
    }

    /**
     * Perform closed loop control on elevator and wrist
     * @return
     */
    public boolean update() {
        currentPosition = liftMaster.getCurrentPosition();
        boolean inPosition = false;

        // Run the elevator motor with 4 different speed zones.  Two up and two down.
        if (liftActive) {
            double error = target - getPosition();
            if (error > DEAD_BAND * 4) {
                // elevator is way too low
                setPower(FAST_LIFT);
            } else if (error > DEAD_BAND) {
                // elevator is little too low
                setPower(SLOW_LIFT);
            } else if (error < -DEAD_BAND * 4) {
                // elevator is way too High
                setPower(FAST_LOWER);
            } else if (error < -DEAD_BAND) {
                // elevator is little too High
                setPower(SLOW_LOWER);
            } else {
                // We are in position, so apply a little hold power unless we are at rest on support
                if (target < 10)
                    setPower(0);
                else
                    setPower(HOLD_POWER);

                inPosition = true;
            }

            // Adjust the angle of the servo.
            // first calculate arm angle and negate it for level wrist
            // Then add desired wrist offset angle and send to servo.
            double armAngle = elevatorEncoderToAngle(currentPosition);
            double servoAngle = -armAngle;
            servoAngle += wristOffset;
            double servoPosition = wristAngleToServo(servoAngle);
            setWristPosition(servoPosition);

            // Display key arm data
            myOpMode.telemetry.addData("arm encoder", currentPosition);
            myOpMode.telemetry.addData("arm angle", armAngle);
            myOpMode.telemetry.addData("servo position", servoPosition);
        }
        return inPosition;
    }

    /***
     * Re-learn the elevator home position.
     * Lower the elevator until it stops and then reset zero position.
     */
    public void recalibrateHomePosition() {
        disableLift();  // Stop any closed loop control
        lastPosition = liftMaster.getCurrentPosition();
        setPower(HOME_POWER);
        myOpMode.sleep(250);

        while (!myOpMode.isStopRequested() && (liftMaster.getCurrentPosition() != lastPosition)){
            lastPosition = liftMaster.getCurrentPosition();
            myOpMode.sleep(100);
        }

        setPower(0);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myOpMode.sleep(50);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setTarget(0);
        setHandPosition(HAND_OPEN);
        enableLift();  // Start closed loop control
    }

    // ----- Elevator controls
    public void enableLift() {
        liftActive = true;
    }

    public void disableLift() {
        liftActive = false;
    }

    public void setTarget(int   target){
        // ToDo:  Make sure the target is a safe value.
        this.target = target;
    }

    public int getTarget(){
        return target;
    }

    public int getPosition() {
        return currentPosition;
    }

    public void jogElevator(double speed) {
        target = target + (int)(speed * 6);
    }

    // ----- Wrist controls
    public void setWristOffset(double angle){
        wristOffset = angle;
    }

    public void setWristPosition(double angle) {
        wrist.setPosition(angle);
    }

    public void setHandPosition(double angle) {
        hand.setPosition(angle);
    }

    // ------ Angle/reading conversions
    public double elevatorEncoderToAngle (int position) {
        return(((position * ENCODER_TO_ANGLEM) + ENCODER_TO_ANGLEC));
    }

    public int elevatorAngleToEncoder (double angle) {
        return((int)((angle-ENCODER_TO_ANGLEC)/ENCODER_TO_ANGLEM));
    }

    public double wristAngleToServo (double angle) {
        return(((angle * WRIST_TO_SERVOM ) + WRIST_TO_SERVOC));
    }

    // ------- Bulk motor control methods
    public void setPower(double power){
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

}


package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ElevatorState.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.List;

public class Elevator {

    // Elevator Constants
    final boolean DROP_ON_HOME = true;
    final double HOME_POWER = -0.1;

    final static int ELEVATOR_MIN    = 0;
    final static  int ELEVATOR_HOME   = 42;
    final static  int ELEVATOR_STACK_TOP = 200;
    final static  int ELEVATOR_LOW    = 460;
    final static  int ELEVATOR_MID    = 710;
    final static  int ELEVATOR_HIGH   = 1000;
    final static  int ELEVATOR_MAX    = 1100;

    final static  int ELEVATOR_TOP_LEVEL  = 4;
    final static  int elevatorLevel[] = { Elevator.ELEVATOR_HOME,
            Elevator.ELEVATOR_STACK_TOP,
            Elevator.ELEVATOR_LOW,
            Elevator.ELEVATOR_MID,
            Elevator.ELEVATOR_HIGH  };

    final int    DEAD_BAND    = 5;
    final double FAST_LIFT    =  0.5;
    final double SLOW_LIFT    =  0.3;
    final double SLOW_LOWER   =  0.0;
    final double FAST_LOWER   = -0.1;
    final double HOLD_POWER   =  0.05;
    final double IN_POSITION_LIMIT = 15;

    // Wrist & Hand Constants
    final double WRIST_UP_OFFSET   = 90;
    final double WRIST_SAFE_OFFSET = 75;

    final double WRIST_HOME_POSITION = 0.56;
    final double HAND_HOME_POSITION = 0.7;

    final double HAND_OPEN  = 0.6; //was 0.47
    final double HAND_READY = 0.7 ;
    final double HAND_CLOSE = 0.95;

    final double WRIST_A2P_M = 0.00374;
    final double WRIST_A2P_C = 0.472;

    // class members (devices)
    private DcMotorEx       liftMaster;
    private DcMotorEx       liftSlave;
    private List<DcMotorEx> motors;
    private Servo           wrist;  //smaller values tilt the wrist down
    private Servo           hand;   //smaller values open the hand more
    private ElapsedTime     elevatorStateTimer = new ElapsedTime();
    private ElapsedTime     runTime = new ElapsedTime();

    // class members (objects)
    private LinearOpMode myOpMode = null;
    private boolean      isAutonomous = false;

    // elevator state variables
    private int     currentElevatorLevel = 0;
    private ElevatorState elevatorState = IDLE;
    private boolean liftActive          = false;
    private boolean liftInPosition      = false;
    private int     liftPosition        = 0;
    private double  liftAngle           = 0;
    private int     liftTargetPosition  = 0;
    private int     liftLastPosition    = 0;
    private boolean wristIsSafe         = false;

    private boolean newLevelReqested = false;
    private int     requestedPosition;
    private double  pendingDelay;
    private int     pendingLiftPosition;
    private ElevatorState pendingState;

    public  boolean grabRequest;
    public  boolean releaseRequest;
    public  boolean handIsOpen = true;

    private double  wristOffset = 0;
    private double  wristPosition = 0;
    private double  wristAngle = 0;
    private double  handPosition = 0;

    private double lift_P2A_m;
    private double lift_P2A_c;

    // Elevator Constructor.  Call once opmode is running.
    public Elevator(LinearOpMode opMode, boolean isAuto) {
        // Attach to hardware devices
        myOpMode = opMode;
        isAutonomous = isAuto;
        liftMaster = myOpMode.hardwareMap.get(DcMotorEx.class, "lift_master");
        liftSlave = myOpMode.hardwareMap.get(DcMotorEx.class, "lefte");
        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");
        hand = myOpMode.hardwareMap.get(Servo.class, "hand");
        motors = Arrays.asList(liftMaster, liftSlave);

        // Angle (A) to/from Position (P) conversion factors Y = Mx + C
        SolveFor p2a = new SolveFor(41, -25, 976, 59);
        lift_P2A_m = p2a.getM();
        lift_P2A_c = p2a.getC();

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMaster.setDirection(DcMotorSimple.Direction.FORWARD);
        liftSlave.setDirection(DcMotorSimple.Direction.REVERSE);
        setHandPosition(HAND_HOME_POSITION);
        setWristPosition(WRIST_HOME_POSITION);

        currentElevatorLevel = 0;
        newLevelReqested = false;
        elevatorState = SharedStates.elevatorState;

        // do any grabber required initializations
        switch (elevatorState) {
            case HOME_OPEN:
                setHandPosition(HAND_OPEN);
                break;

            case HOME_CLOSED:
                myOpMode.sleep(500);
                setHandPosition(HAND_CLOSE);
                break;

            default:
        }

    }

    // ======  Elevator State Machine
    public ElevatorState runStateMachine(ElevatorState newState) {
        setState(newState);
        switch (elevatorState) {
            case IDLE: {
                setState(HOMING);
                break;
            }

            case HOMING: {
                myOpMode.telemetry.addData("Elevator", "Homing");
                myOpMode.telemetry.update();
                recalibrateHomePosition();
                myOpMode.telemetry.addData("Elevator", "Home Done.");
                myOpMode.telemetry.update();
                if (isAutonomous) {
                    setHandPosition(HAND_CLOSE);
                    setState(HOME_CLOSED);
                } else {
                    setHandPosition(HAND_OPEN);
                    setState(HOME_OPEN);
                }
                break;
            }

            case HOME_OPEN: {
               if  (newLiftPosition()) {
                   if (wristIsSafe) {
                       setWristOffset(0);
                   }
                   setLiftTargetPosition(requestedPosition);
                   setState(MOVING_OPEN);
               } else if (grabRequested()) {
                   setHandDelayMove(HAND_CLOSE, 0.3, (liftPosition + 200), HOME_CLOSED);
               } else if (homeRequested()) {
                   if (wristIsSafe) {
                       setWristOffset(0);
                   }
                   setHandPosition(HAND_OPEN);
                   setLiftTargetPosition(ELEVATOR_HOME);
               }
               break;
            }

            case HOME_CLOSED: {
                if  (newLiftPosition()) {
                    setLiftTargetPosition(requestedPosition);
                    setState(MOVING_CLOSED);
                } else if (releaseRequested()) {
                    setHandPosition(HAND_OPEN);
                    setState(HOME_OPEN);
                } else if (homeRequested()) {
                    setLiftTargetPosition(ELEVATOR_HOME);
                    setHandPosition(HAND_OPEN);
                    setState(HOME_OPEN);
                }
                break;
            }

            case GOING_HOME_OPEN: {
                if (liftInPosition) {
                    if (wristIsSafe) {
                        setWristOffset(WRIST_UP_OFFSET);
                    }
                    setHandPosition(HAND_OPEN);
                    currentElevatorLevel = 0;  // Se to home level.
                    setState(HOME_OPEN);
                } else {
                    setWristOffset(WRIST_SAFE_OFFSET);
                }
                break;
            }

            case WAITING_TO_MOVE:{
                // wait for timer to elapse then go to new position and state.
                // Get here by calling setWristDelayMove()
                if (elevatorStateTimer.time() >= pendingDelay) {
                    setLiftTargetPosition(pendingLiftPosition);
                    setState(pendingState);
                }
                break;
            }

            case MOVING_OPEN: {
                if (liftInPosition) {
                    setState(IN_POSITION_OPEN);
                } else if (newLiftPosition()) {
                    setLiftTargetPosition(requestedPosition);
                }
                break;
            }

            case IN_POSITION_OPEN: {
                 if (homeRequested()) {
                     setHandDelayMove(HAND_READY, 0.1, ELEVATOR_HOME, GOING_HOME_OPEN);
                 } else if (grabRequested()) {
                     setHandDelayMove(HAND_CLOSE, 0.3, (liftPosition + 240), IN_POSITION_CLOSED);
                 }  else if (newLiftPosition()) {
                     setLiftTargetPosition(requestedPosition);
                 }

                break;
            }

            case MOVING_CLOSED: {
                if (liftInPosition) {
                    setState(IN_POSITION_CLOSED);
                } else if (newLiftPosition()) {
                    setLiftTargetPosition(requestedPosition);
                }
                break;
            }

            case IN_POSITION_CLOSED: {
                 if (releaseRequested()) {
                     setHandPosition(HAND_OPEN);
                     setState(IN_POSITION_OPEN);
                 } else if (newLiftPosition()) {
                     setLiftTargetPosition(requestedPosition);
                     setState(MOVING_CLOSED);
                 } else if (homeRequested()) {
                     if (DROP_ON_HOME) {
                         setHandDelayMove(HAND_READY, 0.1, ELEVATOR_HOME, GOING_HOME_OPEN);
                     } else {
                         currentElevatorLevel = 0;  // Se to home level.
                         setLiftTargetPosition(ELEVATOR_HOME);
                         setState(HOME_CLOSED);
                     }
                 }
                 break;
            }

            case LOWERING_TO_RELEASE_IN_AUTO: {
                if (liftInPosition) {
                    setHandPosition(HAND_OPEN);
                    setState(RELEASING);
                }
                break;
            }

            case RELEASING: {
                if (elevatorStateTimer.time() > 0.5) {
                    setWristOffset(WRIST_SAFE_OFFSET);
                    setState(FLIPPING_UP);
                }
                break;
            }

            case FLIPPING_UP: {
                if (elevatorStateTimer.time() > 0.5) {
                    setLiftTargetPosition(ELEVATOR_STACK_TOP);
                    setState(IN_POSITION_OPEN);
                }
                break;
            }
        }
        return elevatorState;
    }

    /*
    ----------PILOT/CO-PILOT-------------
    ----------ELEVATOR CONTROLS----------
     */

    /***
     * This is the normal way to run the Elevator state machine.
     * Only use the parameter form if you want to force a state change
     * @return current state.
     */
    public ElevatorState runStateMachine() {
        return runStateMachine(elevatorState);
    }

    public void setState(ElevatorState newState) {
        if (newState != elevatorState) {
            elevatorState = newState;
            elevatorStateTimer.reset();
            SharedStates.elevatorState = newState;
        }
    }

    public String getStateText() {
        return elevatorState.toString();
    }


    private void setHandDelayMove(double handPosition, double delaySec,  int elevatorPosition, ElevatorState nextState) {
        setHandPosition(handPosition);
        pendingDelay = delaySec;
        pendingLiftPosition = elevatorPosition;
        pendingState = nextState;

        setState(WAITING_TO_MOVE);
    }

    /**
     * Perform closed loop control on elevator and wrist
     * @return true if lift is In Position
     */
    public boolean update() {
        liftPosition = liftMaster.getCurrentPosition();

        // Run the elevator motor with 4 different speed zones.  Two up and two down.
        if (liftActive) {
            double error = liftTargetPosition - getLiftPosition();
            if (error > DEAD_BAND * 4) {
                // elevator is way too low
                rampPower(FAST_LIFT);
            } else if (error > DEAD_BAND) {
                // elevator is little too low
                setPower(SLOW_LIFT);
            } else if (error < -DEAD_BAND * 4) {
                // elevator is way too High
                rampPower(FAST_LOWER);
            } else if (error < -DEAD_BAND) {
                // elevator is little too High
                setPower(SLOW_LOWER);
            } else {
                // We are in position, so apply a little hold power unless we are at rest on support
                if (liftTargetPosition <= ELEVATOR_HOME)
                    setPower(0);
                else
                    setPower(HOLD_POWER);

            }

            liftInPosition = (Math.abs(error) <= IN_POSITION_LIMIT);

            // Adjust the angle of the servo.
            // first calculate arm angle and negate it for level wrist
            // Then add desired wrist offset angle and send to servo.
            liftAngle = elevatorEncoderToAngle(liftPosition);
            wristAngle = wristOffset - liftAngle;
            wristPosition = wristAngleToServo(wristAngle);
            setWristPosition(wristPosition);
        }
        return liftInPosition;
    }

    public void showElevatorState() {
        // Display key arm data
        myOpMode.telemetry.addData("arm position", liftPosition);
        myOpMode.telemetry.addData("arm setpoint", liftTargetPosition);
        myOpMode.telemetry.addData("armLevel",     currentElevatorLevel);
        myOpMode.telemetry.addData("arm angle", liftAngle);
        myOpMode.telemetry.addData("servo position", wristPosition);
    }
    /***
     * Start the power off slowly when moving a long way
     * @param power
     */
    private void rampPower(double power) {
        double currentPower =  liftMaster.getPower();
        power = currentPower + ((power - currentPower) * 0.4);
        setPower(power);
    }

    /***
     * Re-learn the elevator home position.
     * Lower the elevator until it stops and then reset zero position.
     */
    public void recalibrateHomePosition() {
        disableLift();  // Stop any closed loop control
        liftLastPosition = liftMaster.getCurrentPosition();
        setPower(HOME_POWER);
        myOpMode.sleep(250);

        while (!myOpMode.isStopRequested() && (liftMaster.getCurrentPosition() != liftLastPosition)){
            liftLastPosition = liftMaster.getCurrentPosition();
            myOpMode.sleep(100);
        }

        setPower(0);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myOpMode.sleep(50);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setLiftTargetPosition(0);
        currentElevatorLevel = 0;
        newLevelReqested = false;
        enableLift();  // Start closed loop control
    }

    private boolean newLiftPosition() {
        if (newLevelReqested) {
            newLevelReqested = false;
            return true;
        } else {
            return false;
        }
    }

    private boolean homeRequested() {
        return myOpMode.gamepad2.left_bumper;
    }

    // ----- Elevator controls  --- REQUESTS FROM External sources
    //
    public void levelUp() {
        // Move up if not at top.
        if (currentElevatorLevel < ELEVATOR_TOP_LEVEL) {
            // look to see if we are at home, and if hand is open or closed
            if ((currentElevatorLevel == 0) && (handPosition == HAND_CLOSE)) {
                // Jump past Top of Stack
                currentElevatorLevel = 2;
            } else {
                currentElevatorLevel++;
            }
        }
        requestedPosition = elevatorLevel[currentElevatorLevel];
        newLevelReqested = true;
    }

    public void levelDown() {
        // Move down if not at bottom
        if (currentElevatorLevel > 0) {
            currentElevatorLevel--;
        }
        requestedPosition = elevatorLevel[currentElevatorLevel];
        newLevelReqested = true;
    }

    public void autoRelease() {
        if (currentElevatorLevel > 0) {
            currentElevatorLevel--;
        }
        setLiftTargetPosition(elevatorLevel[currentElevatorLevel]);
        setState(LOWERING_TO_RELEASE_IN_AUTO);
    }

    private boolean grabRequested() {
        return (grabRequest || myOpMode.gamepad2.square);
    }

    private boolean releaseRequested() {
        return (releaseRequest || myOpMode.gamepad2.circle);
    }

    public void enableLift() {
        liftActive = true;
    }

    public void disableLift() {
        liftActive = false;
    }

    public void setLiftTargetPosition(int Position){
        liftTargetPosition = Range.clip(Position, ELEVATOR_MIN, ELEVATOR_MAX);
        liftInPosition = false;  // set this now to prevent the state machine from skipping
    }

    public int getLiftTargetPosition(){
        return liftTargetPosition;
    }

    public int getLiftPosition() {
        return liftPosition;
    }

    public boolean getWristIsSafe() {
        return wristIsSafe;
    }

    public void jogElevator(double speed) {
        setLiftTargetPosition(liftTargetPosition + (int)(speed * 7));
    }

    public void runElevator(double seconds) {
        runTime.reset();
        while (runTime.time() < seconds) {
            update();
            runStateMachine();
        }
    }

    // ----- Wrist controls
    public void setWristOffset(double angle){
        wristOffset = angle;
        wristIsSafe = (wristOffset > 20);
    }

    public double getWristOffset () {return wristOffset;}

    public void setWristPosition(double position) {

        wristPosition = position;
        wrist.setPosition(position);
    }

    public void setHandPosition(double position) {
        handPosition = position;
        handIsOpen = (position < HAND_CLOSE);
        hand.setPosition(position);
    }

    // ------ Angle/reading conversions
    public double elevatorEncoderToAngle (int position) {
        return(((position * lift_P2A_m) + lift_P2A_c));
    }

    public int elevatorAngleToEncoder (double angle) {
        return((int)((angle- lift_P2A_c)/ lift_P2A_m));
    }

    public double wristAngleToServo (double angle) {
        return(((angle * WRIST_A2P_M) + WRIST_A2P_C));
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


package org.firstinspires.ftc.team417_CENTERSTAGE.mechanisms;

import static org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode.DUMPER_SERVO_RESET_POSITION;
import static org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode.DUMPER_SERVO_TILT_POSITION;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;

public class ArmTeleOp {
    ElapsedTime time = new ElapsedTime();

    private Gamepad gamepad2;
    private DcMotor armMotor;
    private Servo dumperServo;

    //where the arm should currently move to.
    private double armGoalLocation;

    //Used to make sure button inputs are only registered once
    private boolean dpadDownPressed = false;

    //Keeps track of if the arm is in stick control mode, or if it is in move to position mode.
    private boolean dpadUpPressed = false;

    //Keeps track of if the arm is in stick control mode, or if it is in move to position mode.
    public static boolean usingStick = true;

    //The PID constants for the arm
    public static double armKp = 0.01, armKi = 0, armKd = 0;
    public static double armAcceleration = 0.001, maxArmVelocity = 100; //The acceleration of the arm every second, and the maxim highest velocity the arm will accelerate to.
    public static double armVelocity, currentArmLocation; //The velocity the code thinks the arm is at.
    public enum armPhases { //The different phases the arm motion profiling can be in.
        accelerating,
        maintainingSpeed,
        decelerating,
        stopped
    }
    public static armPhases armPhase; //Keeps track of the mode the arm motion profiling is in.
    public static double initArmLocation, armAccelerationOverDist; //The arm location at the start of the motion profile. The distance it took for the arm to accelerate.
    private double lastArmProfileTime ;
    private double lastPidTime, lastPidError, cumulativeError; //The time that the PID loop was last run. The error the last time the PID loop was run. The total error of the pid loop every time it was run.

    public ArmTeleOp(Gamepad gamepad2, DcMotor armMotor, Servo dumperServo) {
        //passes necessary API objects in the class.
        this.gamepad2 = gamepad2;
        this.armMotor = armMotor;
        this.dumperServo = dumperServo;

        //Init variables so they don't hold their value from last time the code was run
        armGoalLocation = 0;
        armVelocity = 0;
        currentArmLocation = 0;
        armPhase = armPhases.accelerating;
        initArmLocation = this.armMotor.getCurrentPosition();
        lastArmProfileTime = 0;
        cumulativeError = 0;
    }

    private double armPID(double goalVelocity, double currentVelocity, double Kp, double Ki, double Kd) {

        double currentTime, elapsedTime;
        double error, rateError;
        double output;

        currentTime = time.seconds();
        elapsedTime = currentTime - lastPidTime;

        error = goalVelocity - currentVelocity;

        if (Ki != 0)
            cumulativeError += error * elapsedTime;

        rateError = (error - lastPidError) / elapsedTime;

        output = Kp * error + Ki * cumulativeError + Kd * rateError;

        lastPidError = error;
        lastPidTime = currentTime;

        return output;
    }

    private void armPidMotionProfile(double goalLocation) {
        double currentTime, elapsedTime; //currentTime: The current time sense boot, elapsedTime: the time sense the last loop of the program.
        double epsilonPercentage = 0.01; //The percentage error where the code will consider the value to be 0.

        currentTime = time.seconds(); //Sets current time to the system clock
        elapsedTime = currentTime - lastArmProfileTime; //calculates the time it took to get back to this function.

        //If in stopped phase and the current location is outside of epsilon of the goal, start the movement again and set the phase to accelerating
        if (armPhase == armPhases.stopped && (armMotor.getCurrentPosition() > goalLocation + goalLocation * epsilonPercentage || armMotor.getCurrentPosition() < goalLocation - goalLocation * epsilonPercentage))
            armPhase = armPhases.accelerating;
        else if (armPhase == armPhases.decelerating) { //If in the decelerating phase, decrease the velocity of the arm based on the acceleration.
            armVelocity -= armAcceleration * elapsedTime;
            if (Math.abs(armMotor.getCurrentPosition()) >= goalLocation){
                armPhase = armPhases.stopped;
            }
        } else if (armPhase == armPhases.maintainingSpeed) { //If in the maintaining speed phase, do not change the velocity of the motor and check to see if it should start decelerating
            if (goalLocation - armMotor.getCurrentPosition() <= armAccelerationOverDist) {
                armPhase = armPhases.decelerating;
            }
        } if (armPhase == armPhases.accelerating) { //if in the accelerating phase, increase the speed of the motors until the max velocity has been hit or until halfway through the movement.
        armVelocity += armAcceleration * elapsedTime;
        if (armVelocity > maxArmVelocity) { //If acceleration is complete cap the velocity and switch phase to maintaining speed.
            armVelocity = maxArmVelocity;
            armAccelerationOverDist = armMotor.getCurrentPosition() - initArmLocation; //save the distance it took to accelerate so we know how long it will take to decelerate
            armPhase = armPhases.maintainingSpeed;
        }
        else if (armMotor.getCurrentPosition() > (goalLocation + initArmLocation) / 2.0) { //If halfway through the turn, switch phase to maintaining speed without capping the velocity.
            armAccelerationOverDist = armMotor.getCurrentPosition() - initArmLocation; //save the distance it took to accelerate so we know how long it will take to decelerate
            armPhase = armPhases.maintainingSpeed;
        }
    }

        currentArmLocation += armVelocity * elapsedTime; //The location the arm should be at that will be fed into the PID.

        //send data to FTC dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("phase", armPhase);
        packet.put("armVelocity", armVelocity);
        packet.put("currentPosition", armMotor.getCurrentPosition());
        packet.put("deltaPosition", armMotor.getCurrentPosition() - initArmLocation);
        packet.put("currentArmLocation", currentArmLocation);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);

        armMotor.setPower(armPID(currentArmLocation, armMotor.getCurrentPosition(), armKp, armKi, armKd)); //Give the PID the location the arm should be at and location the arm is at.

        lastArmProfileTime = currentTime; //The time the loop was competed
    }

    //Finds the smallest position in an array that is greater then current location of the mechanism. Returns 0 if no greater positions are found
    //positionsArray: array of positions in encoder ticks, currentPosition: location of mechanism in encoder ticks from init location
    //acceptableError: the amount of difference between the current location and a position in the array where it will still consider itself past the position
    private double findNextPosition(double[] positionsArray, double currentPosition, double acceptableError) {
        double nextPosition = 0;

        for (int i = 0; i < positionsArray.length; i++) {
            if (positionsArray[i] > currentPosition + acceptableError) {
                nextPosition = positionsArray[i];
                break;
            }
        }

        return nextPosition;
    }

    //Finds the largest position in an array that is smaller then current location of the mechanism. Returns 0 if no smaller positions are found
    //positionsArray: array of positions in encoder ticks, currentPosition: location of mechanism in encoder ticks from init location
    //acceptableError: the amount of difference between the current location and a position in the array where it will still consider itself past the position
    private double findLastPosition(double[] positionsArray, double currentPosition, double acceptableError) {
        double lastPosition = 0;

        for (int i = positionsArray.length - 1; i >= 0; i--) {
            if (positionsArray[i] < currentPosition - acceptableError) {
                lastPosition = positionsArray[i];
                break;
            }
        }

        return lastPosition;
    }

    public void armControl() {
        //How much moving the right stick will effect arm speed outside and inside the robot.
        double rStickSensitivity = 1, rStickSensitivityInsideRobot = rStickSensitivity / 3, rStickSensitivityOutsideRobot = rStickSensitivity;

        double[] ArmPositions = new double[] {BaseOpMode.ARM_MOTOR_MIN_POSITION, BaseOpMode.ARM_MOTOR_MAX_POSITION / 2.0,
                //array of arm positions the dpad can move to.
                BaseOpMode.ARM_MOTOR_MAX_POSITION * (3.0/4.0), BaseOpMode.ARM_MOTOR_MAX_POSITION};

        //The current distance the arm is from its init location
        double armCurrentLocation = armMotor.getCurrentPosition();

        //The amount the right stick has been moved
        double rStickTilt = -gamepad2.right_stick_y;

        //Defines a 0.1 deadzone for right stick input; allows a 25-unit error for arm position goals.
        final double rStickDeadZone = 0.1, armPosEpsilon = 25;

        //Locations the dumper should tilt so the arm does not hit the robot.
        final double startUpwardTiltPos = 75, startUpwardResetPos = 1500, endUpwardResetPos = 2000, startDownwardTiltPos = 1500, endDownwardTiltPos = 250;

        //The speed the arm moves while in move to position mode
        final double armMoveToPositionVelocity = 0.5;

        //If the arm is outside the dead zone switch to using stick control
        if (gamepad2.left_stick_y < -rStickDeadZone || gamepad2.left_stick_y > rStickDeadZone) {
            usingStick = true;

        //If dpad up pressed, move arm to next position in array with armPosEpsilon tolerance.
        } else if (gamepad2.dpad_up && !dpadUpPressed) {
            armGoalLocation = findNextPosition(ArmPositions, armCurrentLocation, armPosEpsilon);
            usingStick = false;

        //If dpad down pressed, move arm to last position in array with armPosEpsilon tolerance.
        } else if (gamepad2.dpad_down && !dpadDownPressed) {
            armGoalLocation = findLastPosition(ArmPositions, armCurrentLocation, armPosEpsilon);
            usingStick = false;
        }

        //If arm is inside robot decrease the speed so arm can't hit robot.
        if (armMotor.getCurrentPosition() < 150.0)
            rStickSensitivity = rStickSensitivityInsideRobot;
        else
            rStickSensitivity = rStickSensitivityOutsideRobot;

        //If using stick control update the speed of arm based on the stick tilt
        if (usingStick) {
            armMotor.setPower(rStickTilt * rStickSensitivity);

            //If stick is inside dead zone set motor power to zero
            if (rStickTilt < rStickDeadZone && rStickTilt > -rStickDeadZone)
                armMotor.setPower(0);
        } else {
            //if the arm is within 50 encoder ticks of it's goal, stop it from moving.
            if (armMotor.getCurrentPosition() < armGoalLocation + armPosEpsilon && armMotor.getCurrentPosition() > armGoalLocation - armPosEpsilon) {
                armMotor.setPower(0);

            //If the arm is past its goal, move backward.
            } else if (armMotor.getCurrentPosition() > armGoalLocation)
                armMotor.setPower(-armMoveToPositionVelocity);

            //If the arm is before it's goal, move forward.
            else if (armMotor.getCurrentPosition() < armGoalLocation)
                armMotor.setPower(armMoveToPositionVelocity);
        }

        if ((armMotor.getCurrentPosition() > startUpwardTiltPos && armMotor.getCurrentPosition() < startUpwardResetPos) && armMotor.getPower() > 0)
            dumperServo.setPosition(DUMPER_SERVO_TILT_POSITION);
        else if (armMotor.getCurrentPosition() < endUpwardResetPos && armMotor.getPower() > 0)
            dumperServo.setPosition(DUMPER_SERVO_RESET_POSITION);

        if (armMotor.getCurrentPosition() < endDownwardTiltPos && armMotor.getPower() < 0)
            dumperServo.setPosition(DUMPER_SERVO_RESET_POSITION);
        else if (armMotor.getCurrentPosition() < startDownwardTiltPos && armMotor.getPower() < 0)
            dumperServo.setPosition(DUMPER_SERVO_TILT_POSITION);


        dpadDownPressed = gamepad2.dpad_down;
        dpadUpPressed = gamepad2.dpad_up;
    }
}
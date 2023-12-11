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

public class ArmMechanism {
    ElapsedTime time = new ElapsedTime();

    private Gamepad gamepad2;
    private DcMotor armMotor;
    private Servo dumperServo;

    private double armGoalLocation;  //where the arm should currently move to.
    private boolean dpadDownPressed = false; //Used to make sure button inputs are only registered once
    private boolean dpadUpPressed = false; //Keeps track of if the arm is in stick control mode, or if it is in move to position mode.
    public static boolean usingStick = true; //Keeps track of if the arm is in stick control mode, or if it is in move to position mode.
    public static double armKp = 0.01, armKi = 0, armKd = 0; //The PID constants for the arm
    public static double armAcceleration = 0.001, maxArmVelocity = 100; //The acceleration of the arm every second, and the maxim highest velocity the arm will accelerate to.
    public static double armVelocity, currentArmLocation; //The velocity the code thinks the arm is at.
    public enum armPhases { //The different phases the arm motion profiling can be in.
        accelerating,
        maintainingSpeed,
        decelerating,
        stopped
    }

    //Keeps track of the mode the arm motion profiling is in.
    public static armPhases armPhase;

    //Arm location at motion profile start, distance for arm acceleration.
    public static double initArmLocation, armAccelerationOverDist;
    private double lastArmProfileTime ;

    //PID last runtime, last error, total error every run iteration.
    private double lastPidTime, lastPidError, cumulativeError;

    public ArmMechanism(Gamepad gamepad2, DcMotor armMotor, Servo dumperServo) {
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
        //currentTime: Time since boot, elapsedTime: Time since last program loop.
        double currentTime, elapsedTime;
        //The percentage error where the code will consider the value to be 0.
        double epsilonPercentage = 0.01;

        //Sets current time to the system clock
        currentTime = time.seconds();
        //calculates the time it took to get back to this function.
        elapsedTime = currentTime - lastArmProfileTime;

        //If stopped and outside epsilon, restart movement, set phase to accelerate.
        if (armPhase == armPhases.stopped && (armMotor.getCurrentPosition() > goalLocation + goalLocation * epsilonPercentage || armMotor.getCurrentPosition() < goalLocation - goalLocation * epsilonPercentage))
            armPhase = armPhases.accelerating;
            //In deceleration phase, decrease arm velocity based on acceleration.
        else if (armPhase == armPhases.decelerating) {
            armVelocity -= armAcceleration * elapsedTime;
            if (Math.abs(armMotor.getCurrentPosition()) >= goalLocation){
                armPhase = armPhases.stopped;
            }
            //
            //In maintaining speed phase, maintain motor velocity and check if deceleration should start.
        } else if (armPhase == armPhases.maintainingSpeed) {
            if (goalLocation - armMotor.getCurrentPosition() <= armAccelerationOverDist) {
                armPhase = armPhases.decelerating;
            }
            //If in acceleration phase, increase motor speed until max velocity or halfway through movement.
        } if (armPhase == armPhases.accelerating) {
        armVelocity += armAcceleration * elapsedTime;
            //If acceleration is complete cap the velocity and switch phase to maintaining speed.
        if (armVelocity > maxArmVelocity) {
            armVelocity = maxArmVelocity;
            //save the distance it took to accelerate so we know how long it will take to decelerate
            armAccelerationOverDist = armMotor.getCurrentPosition() - initArmLocation;
            armPhase = armPhases.maintainingSpeed;
        }
        //If halfway through the turn, switch phase to maintaining speed without capping the velocity.
        else if (armMotor.getCurrentPosition() > (goalLocation + initArmLocation) / 2.0) {
            //save the distance it took to accelerate so we know how long it will take to decelerate
            armAccelerationOverDist = armMotor.getCurrentPosition() - initArmLocation;
            armPhase = armPhases.maintainingSpeed;
        }
    }
//The location the arm should be at that will be fed into the PID.
        currentArmLocation += armVelocity * elapsedTime;

        //send data to FTC dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("phase", armPhase);
        packet.put("armVelocity", armVelocity);
        packet.put("currentPosition", armMotor.getCurrentPosition());
        packet.put("deltaPosition", armMotor.getCurrentPosition() - initArmLocation);
        packet.put("currentArmLocation", currentArmLocation);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);
//Give the PID the location the arm should be at and location the arm is at.
        armMotor.setPower(armPID(currentArmLocation, armMotor.getCurrentPosition(), armKp, armKi, armKd));

        //The time the loop was competed
        lastArmProfileTime = currentTime;
    }

    //Finds largest position in array smaller than current location. Returns 0 if none found.
    //positionsArray: encoder tick positions, currentPosition: mechanism location in ticks from init.
    //acceptableError: allowable diff. between current location and array position to be considered past it.
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

    //Finds largest position in array smaller than current location. Returns 0 if none found.
    //positionsArray: encoder tick positions, currentPosition: mechanism location in ticks from init.
    //acceptableError: allowable diff. between current location and array position to be considered past it.
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
        double rStickSensitivity = 1, rStickSensitivityInsideRobot = rStickSensitivity / 3, rStickSensitivityOutsideRobot = rStickSensitivity; //How much moving the right stick will effect arm speed outside and inside the robot.
        double[] ArmPositions = new double[] {BaseOpMode.ARM_MOTOR_MIN_POSITION, BaseOpMode.ARM_MOTOR_MAX_POSITION / 2.0,
                BaseOpMode.ARM_MOTOR_MAX_POSITION * (3.0/4.0), BaseOpMode.ARM_MOTOR_MAX_POSITION}; //array of arm positions the dpad can move to.
        double armCurrentLocation = armMotor.getCurrentPosition(); //The current distance the arm is from its init location
        double rStickTilt = -gamepad2.right_stick_y; //The amount the right stick has been moved
        final double rStickDeadZone = 0.1, armPosEpsilon = 25; //rStickDeadZone: the amount of space that is considered zero, armPosEpsilon: the amount of error between the current location and the goal location that is acceptable.
        final double startUpwardTiltPos = 75, startUpwardResetPos = 1500, endUpwardResetPos = 2000, startDownwardTiltPos = 1500, endDownwardTiltPos = 250; //Locations the dumper should tilt so the arm does not hit the robot.
        final double armMoveToPositionVelocity = 0.5; //The speed the arm moves while in move to position mode

        if (gamepad2.left_stick_y < -rStickDeadZone || gamepad2.left_stick_y > rStickDeadZone) { //If the arm is outside the dead zone switch to using stick control
            usingStick = true;
        } else if (gamepad2.dpad_up && !dpadUpPressed) { //if dpad up is being pressed change the arm location to the next location in the arm positions array.
            armGoalLocation = findNextPosition(ArmPositions, armCurrentLocation, armPosEpsilon);
            usingStick = false;
        } else if (gamepad2.dpad_down && !dpadDownPressed) { //if dpad down is being pressed change the arm location to the last location in the arm positions array.
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
            } else if (armMotor.getCurrentPosition() > armGoalLocation) //If the arm is past it's goal, move backward.
                armMotor.setPower(-armMoveToPositionVelocity);
            else if (armMotor.getCurrentPosition() < armGoalLocation) //If the arm is before it's goal, move forward.
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
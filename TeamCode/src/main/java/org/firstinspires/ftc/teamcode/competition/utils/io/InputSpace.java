package org.firstinspires.ftc.teamcode.competition.utils.io;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.competition.utils.locations.DuckMotorLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.ElevatorLeftLiftMotorLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.ElevatorRightLiftMotorLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.HandGrabbingServoLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.HandSpinningServoXLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.HandSpinningServoYLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.IntakeLiftingServoLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.IntakeSpinningMotorLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.TankDrivetrainLocation;

/**
 * This class can be used to send input to locations. Locations then attempt to handle the input. For example, if the location of motor $x receives a value, the location will attempt to spin that motor at $x speed. If it fails, it will consume any exceptions and act like nothing happened. When creating an InputSpace, the InputSpace attempts to create locations for every input location on the robot. For example, if the team decides to build a robot with 4 motors, locations for all 4 will be created and able to send data to. The possible locations are defined inside this class.
 * @implSpec Since the InputSpace is designed to handle input for the current robot, it should be built with the current robot and only the current robot in mind. Location classes are where all used locations should go. If they're unused on the current robot, leave them there. They should never be removed because they could be used later, and it's not like they really take up any extra resources anyway.
 */
public class InputSpace {

    private final HardwareMap HARDWARE;
    private final TankDrivetrainLocation TANK;
    private final DuckMotorLocation DUCK;
    private final ElevatorLeftLiftMotorLocation ELEVATOR_LEFT;
    private final ElevatorRightLiftMotorLocation ELEVATOR_RIGHT;
    private final HandGrabbingServoLocation HAND_GRABBING_SERVO;
    private final HandSpinningServoXLocation HAND_SPINNING_SERVO_X;
    private final HandSpinningServoYLocation HAND_SPINNING_SERVO_Y;
    private final IntakeLiftingServoLocation INTAKE_LIFTING_SERVO;
    private final IntakeSpinningMotorLocation INTAKE_SPINNING_MOTOR;

    public InputSpace(HardwareMap hardware) {
        HARDWARE = hardware;
        TANK = new TankDrivetrainLocation(HARDWARE);
        DUCK = new DuckMotorLocation(HARDWARE);
        ELEVATOR_LEFT = new ElevatorLeftLiftMotorLocation(HARDWARE);
        ELEVATOR_RIGHT = new ElevatorRightLiftMotorLocation(HARDWARE);
        HAND_GRABBING_SERVO = new HandGrabbingServoLocation(HARDWARE);
        HAND_SPINNING_SERVO_X = new HandSpinningServoXLocation(HARDWARE);
        HAND_SPINNING_SERVO_Y = new HandSpinningServoYLocation(HARDWARE);
        INTAKE_LIFTING_SERVO = new IntakeLiftingServoLocation(HARDWARE);
        INTAKE_SPINNING_MOTOR = new IntakeSpinningMotorLocation(HARDWARE);
    }

    public void sendInputToTank(TankDrivetrainLocation.Action action, int rightInput, int leftInput) {
        TANK.handleInput(action, rightInput, leftInput);
    }

    public void sendInputToDuckMotor(DuckMotorLocation.Action action, int input) {
        DUCK.handleInput(action, input);
    }

    public void sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action action, int input) {
        ELEVATOR_LEFT.handleInput(action, input);
    }

    public void sendInputToElevatorRightLift(ElevatorRightLiftMotorLocation.Action action, int input) {
        ELEVATOR_RIGHT.handleInput(action, input);
    }

    public void sendInputToHandGrabber(HandGrabbingServoLocation.Action action, int input) {
        HAND_GRABBING_SERVO.handleInput(action, input);
    }

    public void sendInputToHandSpinnerOnTheX(HandSpinningServoXLocation.Action action, int input) {
        HAND_SPINNING_SERVO_X.handleInput(action, input);
    }

    public void sendInputToHandSpinnerOnTheY(HandSpinningServoYLocation.Action action, int input) {
        HAND_SPINNING_SERVO_Y.handleInput(action, input);
    }

    public void sendInputToIntakeLifter(IntakeLiftingServoLocation.Action action, int input) {
        INTAKE_LIFTING_SERVO.handleInput(action, input);
    }

    public void sendInputToIntakeSpinner(IntakeSpinningMotorLocation.Action action, int input) {
        INTAKE_SPINNING_MOTOR.handleInput(action, input);
    }

    public HardwareMap getHardwareMap() {
        return HARDWARE;
    }

    public TankDrivetrainLocation getTank() {
        return TANK;
    }

    public DuckMotorLocation getDuckMotor() {
        return DUCK;
    }

    public ElevatorLeftLiftMotorLocation getElevatorLeftLift() {
        return ELEVATOR_LEFT;
    }

    public ElevatorRightLiftMotorLocation getElevatorRightLift() {
        return ELEVATOR_RIGHT;
    }

    public HandGrabbingServoLocation getHandGrabber() {
        return HAND_GRABBING_SERVO;
    }

    public HandSpinningServoXLocation getHandSpinnerOnTheX() {
        return HAND_SPINNING_SERVO_X;
    }

    public HandSpinningServoYLocation getHandSpinnerOnTheY() {
        return HAND_SPINNING_SERVO_Y;
    }

    public IntakeLiftingServoLocation getIntakeLifter() {
        return INTAKE_LIFTING_SERVO;
    }

    public IntakeSpinningMotorLocation getIntakeSpinner() {
        return INTAKE_SPINNING_MOTOR;
    }

    public void stop() {
        TANK.stop();
        DUCK.stop();
        ELEVATOR_LEFT.stop();
        ELEVATOR_RIGHT.stop();
        HAND_GRABBING_SERVO.stop();
        HAND_SPINNING_SERVO_X.stop();
        HAND_SPINNING_SERVO_Y.stop();
        INTAKE_LIFTING_SERVO.stop();
        INTAKE_SPINNING_MOTOR.stop();
    }

}

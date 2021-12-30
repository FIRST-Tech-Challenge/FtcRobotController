package org.firstinspires.ftc.teamcode.main.scripts.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.main.utils.gamepads.GamepadManager;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardMotor;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardServo;
import org.firstinspires.ftc.teamcode.main.utils.io.InputSpace;
import org.firstinspires.ftc.teamcode.main.utils.io.OutputSpace;
import org.firstinspires.ftc.teamcode.main.utils.locations.ElevatorBottomLimitSwitchLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.ElevatorLeftLiftMotorLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.ElevatorRightLiftMotorLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.HandGrabbingServoLeftLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.HandGrabbingServoRightLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.IntakeLiftingServoLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.IntakeSpinningMotorLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.TankDrivetrainLocation;
import org.firstinspires.ftc.teamcode.main.utils.scripting.TeleOpScript;

public class FullTeleOpScript extends TeleOpScript {


    /*

    MR. ROCHE - READ ME

    Any comment that starts with // ROCHE in this file says something you should know to run this yourself.

     */


    private GamepadManager gamepadManager;
    private double timeAsOfLastIntakeMovement;
    private int intakeLowerPos, intakeUpperPos;
    private boolean intakeShouldBeDown, intakeIsAtPosition;
    private InputSpace inputSpace;
    private OutputSpace outputSpace;

    public FullTeleOpScript(LinearOpMode opMode) {
        // set values
        super(opMode);
        inputSpace = new InputSpace(getOpMode().hardwareMap);
        outputSpace = new OutputSpace(getOpMode().hardwareMap);
        gamepadManager = new GamepadManager(getOpMode().gamepad1, getOpMode().gamepad1, getOpMode().gamepad1, getOpMode().gamepad1, getOpMode().gamepad1, getOpMode().gamepad1);
        timeAsOfLastIntakeMovement = 0;
        intakeLowerPos = 10;
        intakeUpperPos = 30;
        intakeShouldBeDown = false;
        intakeIsAtPosition = false;
        // calibrate elevator
        // ROCHE - you can uncomment this code to make the elevator calibrate itself when it starts up, it requires both elevator motors and the limit switch to be working though
//        int timeAsOfLastElevatorCalibrationBegin = (int) getOpMode().time;
//        while(outputSpace.receiveOutputFromElevatorBottomLimitSwitch(ElevatorBottomLimitSwitchLocation.Values.PRESSED) == 0 && timeAsOfLastElevatorCalibrationBegin > (int) getOpMode().time - 1) {
//            inputSpace.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.SET_SPEED, 100);
//            inputSpace.sendInputToElevatorRightLift(ElevatorRightLiftMotorLocation.Action.SET_SPEED, -100);
//        }
//        while(outputSpace.receiveOutputFromElevatorBottomLimitSwitch(ElevatorBottomLimitSwitchLocation.Values.PRESSED) == 0) {
//            inputSpace.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.SET_SPEED, -10);
//            inputSpace.sendInputToElevatorRightLift(ElevatorRightLiftMotorLocation.Action.SET_SPEED, 10);
//        }
        ((StandardMotor) inputSpace.getElevatorLeftLift().getInternalInteractionSurface()).reset();
        ((StandardMotor) inputSpace.getElevatorRightLift().getInternalInteractionSurface()).reset();
        // calibrate hand
        inputSpace.sendInputToHandRightGrabber(HandGrabbingServoRightLocation.Action.SET_POSITION, 100);
        inputSpace.sendInputToHandLeftGrabber(HandGrabbingServoLeftLocation.Action.SET_POSITION, 10);
        opMode.sleep(1000);
        inputSpace.sendInputToHandRightGrabber(HandGrabbingServoRightLocation.Action.SET_POSITION, 83);
        inputSpace.sendInputToHandLeftGrabber(HandGrabbingServoLeftLocation.Action.SET_POSITION, 28);
        opMode.sleep(1000);
        inputSpace.sendInputToHandRightGrabber(HandGrabbingServoRightLocation.Action.SET_POSITION, 100);
        inputSpace.sendInputToHandLeftGrabber(HandGrabbingServoLeftLocation.Action.SET_POSITION, 10);
        // calibrate intake
        // ROCHE - you can uncomment this to make the spinny grabber thing calibrate itself, but it's not tested
//        while(!intakeIsAtPosition) {
//            if(timeAsOfLastIntakeMovement < getOpMode().time - 5 && !intakeIsAtPosition || timeAsOfLastIntakeMovement == 0 && !intakeIsAtPosition) {
//                int pos = ((StandardServo) inputSpace.getIntakeLifter().getInternalInteractionSurface()).getPosition();
//                if((int)outputSpace.receiveOutputFromIntakeLiftingDistanceSensor() < intakeLowerPos) {
//                    inputSpace.sendInputToIntakeLifter(IntakeLiftingServoLocation.Action.SET_POSITION, pos + 5);
//                }else if((int) outputSpace.receiveOutputFromIntakeLiftingDistanceSensor() > intakeLowerPos) {
//                    inputSpace.sendInputToIntakeLifter(IntakeLiftingServoLocation.Action.SET_POSITION, pos - 5);
//                }else{
//                    intakeIsAtPosition = true;
//                }
//                timeAsOfLastIntakeMovement = getOpMode().time;
//            }
//        }
    }

    @Override
    public void main() {
        // update drivetrain
        int left = (int) Range.clip(gamepadManager.functionOneGamepad().left_stick_y * 75, -75, 75);
        int right = (int) Range.clip(gamepadManager.functionOneGamepad().right_stick_y * 75, -75, 75);
        inputSpace.sendInputToTank(TankDrivetrainLocation.Action.SET_SPEED, -right, -left);
        // lift intake if needed
        if(gamepadManager.functionOneGamepad().dpad_right) {
            intakeShouldBeDown = false;
        }else if(gamepadManager.functionOneGamepad().dpad_left) {
            intakeShouldBeDown = true;
        }
        // ROCHE - uncomment this to add functionality to the spinny grabber lifter thingy, untested like the calibration section. You also need to comment out lines 130 and 131 for this. If the spinny thing isnt at the right position while this code is uncommented and 130/131 is commented, it wont spin.
//        if(intakeShouldBeDown) {
//            if(timeAsOfLastIntakeMovement < getOpMode().time - 5 && !intakeIsAtPosition || timeAsOfLastIntakeMovement == 0 && !intakeIsAtPosition) {
//                int pos = ((StandardServo) inputSpace.getIntakeLifter().getInternalInteractionSurface()).getPosition();
//                if((int)outputSpace.receiveOutputFromIntakeLiftingDistanceSensor() < intakeLowerPos) {
//                    inputSpace.sendInputToIntakeLifter(IntakeLiftingServoLocation.Action.SET_POSITION, pos + 5);
//                }else if((int) outputSpace.receiveOutputFromIntakeLiftingDistanceSensor() > intakeLowerPos) {
//                    inputSpace.sendInputToIntakeLifter(IntakeLiftingServoLocation.Action.SET_POSITION, pos - 5);
//                }else{
//                    intakeIsAtPosition = true;
//                }
//                timeAsOfLastIntakeMovement = getOpMode().time;
//            }
//        }else{
//            if(timeAsOfLastIntakeMovement < getOpMode().time - 5 && !intakeIsAtPosition || timeAsOfLastIntakeMovement == 0 && !intakeIsAtPosition) {
//                int pos = ((StandardServo) inputSpace.getIntakeLifter().getInternalInteractionSurface()).getPosition();
//                if((int)outputSpace.receiveOutputFromIntakeLiftingDistanceSensor() < intakeUpperPos) {
//                    inputSpace.sendInputToIntakeLifter(IntakeLiftingServoLocation.Action.SET_POSITION, pos + 5);
//                }else if((int) outputSpace.receiveOutputFromIntakeLiftingDistanceSensor() > intakeUpperPos) {
//                    inputSpace.sendInputToIntakeLifter(IntakeLiftingServoLocation.Action.SET_POSITION, pos - 5);
//                }else{
//                    intakeIsAtPosition = true;
//                }
//                timeAsOfLastIntakeMovement = getOpMode().time;
//            }
//        }
        // debugging
        // FIXME: this is temp, del later
        intakeShouldBeDown = true;
        intakeIsAtPosition = true;
        // update intake motor
        if(intakeShouldBeDown && intakeIsAtPosition) {
            int intakeGas = (int) Range.clip(gamepadManager.functionOneGamepad().left_trigger * 100, 0, 100);
            int intakeBrake = (int) Range.clip(gamepadManager.functionOneGamepad().right_trigger * 100, 0, 100);
            int intakeSpeed = Range.clip(intakeGas - intakeBrake, -100, 100);
            inputSpace.sendInputToIntakeSpinner(IntakeSpinningMotorLocation.Action.SET_SPEED, intakeSpeed);
        }
        // control elevator
        int elevatorInput = (gamepadManager.functionOneGamepad().right_bumper ? 0 : 1) + (gamepadManager.functionOneGamepad().left_bumper ? 0 : -1);
        int inputVal = ((StandardMotor) inputSpace.getElevatorLeftLift().getInternalInteractionSurface()).getDcMotor().getCurrentPosition() > -300 ? Range.clip(elevatorInput * 75, -25, 5) : Range.clip(elevatorInput * 75, -75, 75);
        if(inputVal < 0 || outputSpace.receiveOutputFromElevatorBottomLimitSwitch(ElevatorBottomLimitSwitchLocation.Values.PRESSED) == 0) {
            inputSpace.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.SET_SPEED, inputVal);
            inputSpace.sendInputToElevatorRightLift(ElevatorRightLiftMotorLocation.Action.SET_SPEED, -inputVal);
        }else{
            inputSpace.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.SET_SPEED, 0);
            inputSpace.sendInputToElevatorRightLift(ElevatorRightLiftMotorLocation.Action.SET_SPEED, 0);
        }
        // grab item if it exists
        getOpMode().telemetry.addData("Distance<Hand, Intake>: ", String.valueOf(outputSpace.receiveOutputFromHandDistanceSensor()) + " " + String.valueOf(outputSpace.receiveOutputFromIntakeLiftingDistanceSensor()));
        getOpMode().telemetry.update();
        if(outputSpace.receiveOutputFromHandDistanceSensor() < 60) {
            inputSpace.sendInputToHandRightGrabber(HandGrabbingServoRightLocation.Action.SET_POSITION, 83);
            inputSpace.sendInputToHandLeftGrabber(HandGrabbingServoLeftLocation.Action.SET_POSITION, 28);
        }else{
            inputSpace.sendInputToHandRightGrabber(HandGrabbingServoRightLocation.Action.SET_POSITION, 100);
            inputSpace.sendInputToHandLeftGrabber(HandGrabbingServoLeftLocation.Action.SET_POSITION, 10);
        }
    }

    @Override
    public void stop() {
        inputSpace.stop();
        outputSpace.stop();
    }

}

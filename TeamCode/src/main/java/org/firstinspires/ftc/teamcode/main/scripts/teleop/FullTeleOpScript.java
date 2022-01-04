package org.firstinspires.ftc.teamcode.main.scripts.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.main.utils.gamepads.GamepadManager;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardMotor;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardServo;
import org.firstinspires.ftc.teamcode.main.utils.io.InputSpace;
import org.firstinspires.ftc.teamcode.main.utils.io.OutputSpace;
import org.firstinspires.ftc.teamcode.main.utils.locations.DuckMotorLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.ElevatorBottomLimitSwitchLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.ElevatorLeftLiftMotorLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.ElevatorRightLiftMotorLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.HandSpinningServoLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.IntakeLiftingServoLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.IntakeSpinningMotorLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.TankDrivetrainLocation;
import org.firstinspires.ftc.teamcode.main.utils.scripting.TeleOpScript;

public class FullTeleOpScript extends TeleOpScript {

    private GamepadManager gamepadManager;
    private double timeAsOfLastIntakeMovement;
    private int intakeLowerPos, intakeUpperPos;
    private boolean intakeShouldBeDown, intakeIsAtPosition, handShouldBeDown, bWasDown;
    private InputSpace inputSpace;
    private OutputSpace outputSpace;

    public FullTeleOpScript(LinearOpMode opMode) {
        super(opMode);
        // set fields and calibrate robot
        assignValues();
//        calibrateElevator();
//        calibrateIntake();
    }

    @Override
    public void main() {
        // control robot
        controlDrivetrain();
//        controlIntakeLifter();
        // debug
        intakeShouldBeDown = true; intakeIsAtPosition = true;
        controlIntake();
        controlElevator();
        controlHand();
        controlDuck();
        // debug
        debug();
    }

    private void assignValues() {
        inputSpace = new InputSpace(getOpMode().hardwareMap);
        outputSpace = new OutputSpace(getOpMode().hardwareMap);
        gamepadManager = new GamepadManager(getOpMode().gamepad1, getOpMode().gamepad1, getOpMode().gamepad1, getOpMode().gamepad1, getOpMode().gamepad1, getOpMode().gamepad1);
        timeAsOfLastIntakeMovement = 0;
        intakeLowerPos = 10;
        intakeUpperPos = 30;
        intakeShouldBeDown = false;
        intakeIsAtPosition = false;
        handShouldBeDown = false;
        bWasDown = false;
    }

    private void calibrateElevator() {
        int timeAsOfLastElevatorCalibrationBegin = (int) getOpMode().time;
        while(outputSpace.receiveOutputFromElevatorBottomLimitSwitch(ElevatorBottomLimitSwitchLocation.Values.PRESSED) == 0 && timeAsOfLastElevatorCalibrationBegin > (int) getOpMode().time - 1) {
            inputSpace.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.SET_SPEED, 100);
            inputSpace.sendInputToElevatorRightLift(ElevatorRightLiftMotorLocation.Action.SET_SPEED, 100);
        }
        while(outputSpace.receiveOutputFromElevatorBottomLimitSwitch(ElevatorBottomLimitSwitchLocation.Values.PRESSED) == 0) {
            inputSpace.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.SET_SPEED, -10);
            inputSpace.sendInputToElevatorRightLift(ElevatorRightLiftMotorLocation.Action.SET_SPEED, 10);
        }
        ((StandardMotor) inputSpace.getElevatorLeftLift().getInternalInteractionSurface()).reset();
        ((StandardMotor) inputSpace.getElevatorRightLift().getInternalInteractionSurface()).reset();
    }

    private void calibrateIntake() {
        while(!intakeIsAtPosition) {
            if(timeAsOfLastIntakeMovement < getOpMode().time - 5 && !intakeIsAtPosition || timeAsOfLastIntakeMovement == 0 && !intakeIsAtPosition) {
                int pos = ((StandardServo) inputSpace.getIntakeLifter().getInternalInteractionSurface()).getPosition();
                if((int)outputSpace.receiveOutputFromIntakeLiftingDistanceSensor() < intakeLowerPos) {
                    inputSpace.sendInputToIntakeLifter(IntakeLiftingServoLocation.Action.SET_POSITION, pos + 5);
                }else if((int) outputSpace.receiveOutputFromIntakeLiftingDistanceSensor() > intakeLowerPos) {
                    inputSpace.sendInputToIntakeLifter(IntakeLiftingServoLocation.Action.SET_POSITION, pos - 5);
                }else{
                    intakeIsAtPosition = true;
                }
                timeAsOfLastIntakeMovement = getOpMode().time;
            }
        }
    }

    private void controlDrivetrain() {
        int left = (int) Range.clip(gamepadManager.functionOneGamepad().left_stick_y * 75, -75, 75);
        int right = (int) Range.clip(gamepadManager.functionOneGamepad().right_stick_y * 75, -75, 75);
        inputSpace.sendInputToTank(TankDrivetrainLocation.Action.SET_SPEED, -right, -left);
    }

    private void controlIntakeLifter() {
        if(gamepadManager.functionOneGamepad().dpad_right) {
            intakeShouldBeDown = false;
        }else if(gamepadManager.functionOneGamepad().dpad_left) {
            intakeShouldBeDown = true;
        }
        if(intakeShouldBeDown) {
            if(timeAsOfLastIntakeMovement < getOpMode().time - 5 && !intakeIsAtPosition || timeAsOfLastIntakeMovement == 0 && !intakeIsAtPosition) {
                int pos = ((StandardServo) inputSpace.getIntakeLifter().getInternalInteractionSurface()).getPosition();
                if((int)outputSpace.receiveOutputFromIntakeLiftingDistanceSensor() < intakeLowerPos) {
                    inputSpace.sendInputToIntakeLifter(IntakeLiftingServoLocation.Action.SET_POSITION, pos + 5);
                }else if((int) outputSpace.receiveOutputFromIntakeLiftingDistanceSensor() > intakeLowerPos) {
                    inputSpace.sendInputToIntakeLifter(IntakeLiftingServoLocation.Action.SET_POSITION, pos - 5);
                }else{
                    intakeIsAtPosition = true;
                }
                timeAsOfLastIntakeMovement = getOpMode().time;
            }
        }else{
            if(timeAsOfLastIntakeMovement < getOpMode().time - 5 && !intakeIsAtPosition || timeAsOfLastIntakeMovement == 0 && !intakeIsAtPosition) {
                int pos = ((StandardServo) inputSpace.getIntakeLifter().getInternalInteractionSurface()).getPosition();
                if((int)outputSpace.receiveOutputFromIntakeLiftingDistanceSensor() < intakeUpperPos) {
                    inputSpace.sendInputToIntakeLifter(IntakeLiftingServoLocation.Action.SET_POSITION, pos + 5);
                }else if((int) outputSpace.receiveOutputFromIntakeLiftingDistanceSensor() > intakeUpperPos) {
                    inputSpace.sendInputToIntakeLifter(IntakeLiftingServoLocation.Action.SET_POSITION, pos - 5);
                }else{
                    intakeIsAtPosition = true;
                }
                timeAsOfLastIntakeMovement = getOpMode().time;
            }
        }
    }

    private void controlIntake() {
        if(intakeShouldBeDown && intakeIsAtPosition) {
            int intakeGas = (int) Range.clip(gamepadManager.functionOneGamepad().left_trigger * 100, 0, 100);
            int intakeBrake = (int) Range.clip(gamepadManager.functionOneGamepad().right_trigger * 100, 0, 100);
            int intakeSpeed = Range.clip(intakeGas - intakeBrake, -100, 100);
            inputSpace.sendInputToIntakeSpinner(IntakeSpinningMotorLocation.Action.SET_SPEED, intakeSpeed);
        }
    }

    private void controlElevator() {
        int elevatorInput = (gamepadManager.functionOneGamepad().right_bumper ? 0 : 1) + (gamepadManager.functionOneGamepad().left_bumper ? 0 : -1);
        int inputVal = ((StandardMotor) inputSpace.getElevatorLeftLift().getInternalInteractionSurface()).getDcMotor().getCurrentPosition() > -300 ? Range.clip(elevatorInput * 75, -75, 75) : Range.clip(elevatorInput * 75, -25, 5);
        if(inputVal < 0 || outputSpace.receiveOutputFromElevatorBottomLimitSwitch(ElevatorBottomLimitSwitchLocation.Values.PRESSED) == 0) {
            inputSpace.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.SET_SPEED, inputVal);
            inputSpace.sendInputToElevatorRightLift(ElevatorRightLiftMotorLocation.Action.SET_SPEED, inputVal);
        }else{
            inputSpace.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.SET_SPEED, 0);
            inputSpace.sendInputToElevatorRightLift(ElevatorRightLiftMotorLocation.Action.SET_SPEED, 0);
        }
    }

    private void controlHand() {
        if(gamepadManager.functionOneGamepad().b) {
            if(!bWasDown) {
                handShouldBeDown = !handShouldBeDown;
            }
            bWasDown = true;
        }else{
            bWasDown = false;
        }
        if(handShouldBeDown) {
            inputSpace.sendInputToHandSpinner(HandSpinningServoLocation.Action.SET_POSITION, 20);
        }else{
            inputSpace.sendInputToHandSpinner(HandSpinningServoLocation.Action.SET_POSITION, 50);
        }
    }

    private void controlDuck() {
        inputSpace.sendInputToDuckMotor(DuckMotorLocation.Action.SET_SPEED, getOpMode().gamepad1.a ? 50 : 0);
    }

    private void debug() {
        getOpMode().telemetry.addData("Values<Intake Distance, ElevatorEncoderValues<Left, Right>>: ", "<" + outputSpace.receiveOutputFromIntakeLiftingDistanceSensor() + ", " + "<" + ((StandardMotor) inputSpace.getElevatorLeftLift().getInternalInteractionSurface()).getDcMotor().getCurrentPosition() + ", " + ((StandardMotor) inputSpace.getElevatorRightLift().getInternalInteractionSurface()).getDcMotor().getCurrentPosition() + ">>");
        getOpMode().telemetry.update();
    }

    @Override
    public void stop() {
        inputSpace.stop();
        outputSpace.stop();
    }

}

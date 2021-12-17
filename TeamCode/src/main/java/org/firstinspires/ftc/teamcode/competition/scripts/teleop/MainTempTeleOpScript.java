package org.firstinspires.ftc.teamcode.competition.scripts.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.competition.utils.gamepads.GamepadManager;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardMotor;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardServo;
import org.firstinspires.ftc.teamcode.competition.utils.io.InputSpace;
import org.firstinspires.ftc.teamcode.competition.utils.io.OutputSpace;
import org.firstinspires.ftc.teamcode.competition.utils.locations.ElevatorBottomLimitSwitchLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.ElevatorLeftLiftMotorLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.ElevatorRightLiftMotorLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.IntakeLiftingServoLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.IntakeSpinningMotorLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.TankDrivetrainLocation;
import org.firstinspires.ftc.teamcode.competition.utils.scripting.TeleOpScript;

public class MainTempTeleOpScript extends TeleOpScript {

    private GamepadManager gamepadManager;
    private double timeAsOfLastIntakeMovement;
    private int intakeLowerPos, intakeUpperPos;
    private boolean intakeShouldBeDown, intakeIsAtPosition;
    private InputSpace inputSpace;
    private OutputSpace outputSpace;

    public MainTempTeleOpScript(LinearOpMode opMode) {
        super(opMode);
        inputSpace = new InputSpace(getOpMode().hardwareMap);
        outputSpace = new OutputSpace(getOpMode().hardwareMap);
        gamepadManager = new GamepadManager(getOpMode().gamepad1, getOpMode().gamepad1, getOpMode().gamepad1, getOpMode().gamepad1, getOpMode().gamepad1, getOpMode().gamepad1);
        timeAsOfLastIntakeMovement = 0;
        intakeLowerPos = 10;
        intakeUpperPos = 30;
        intakeShouldBeDown = false;
        intakeIsAtPosition = false;
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

        // driving
        int left = (int) Range.clip(gamepadManager.functionOneGamepad().left_stick_y * 75, -75, 75);
        int right = (int) Range.clip(gamepadManager.functionOneGamepad().right_stick_y * 75, -75, 75);
        inputSpace.sendInputToTank(TankDrivetrainLocation.Action.SET_SPEED, -right, -left);

        // intake lifter
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

        getOpMode().telemetry.addData("Distance: ", outputSpace.receiveOutputFromIntakeLiftingDistanceSensor());
        getOpMode().telemetry.update();

        // intake
        if(intakeShouldBeDown && intakeIsAtPosition) {
            int intakeGas = (int) Range.clip(gamepadManager.functionOneGamepad().left_trigger * 100, 0, 100);
            int intakeBrake = (int) Range.clip(gamepadManager.functionOneGamepad().right_trigger * 100, 0, 100);
            int intakeSpeed = Range.clip(intakeGas - intakeBrake, -100, 100);
            inputSpace.sendInputToIntakeSpinner(IntakeSpinningMotorLocation.Action.SET_SPEED, intakeSpeed);
        }

        // elevator
        int elevatorInput = (gamepadManager.functionOneGamepad().right_bumper ? 0 : 1) + (gamepadManager.functionOneGamepad().left_bumper ? 0 : -1);
        int inputVal = ((StandardMotor) inputSpace.getElevatorLeftLift().getInternalInteractionSurface()).getDcMotor().getCurrentPosition() > -200 ? Range.clip(elevatorInput * 25, -25, 5) : Range.clip(elevatorInput * 25, -25, 25);
        if(inputVal < 0 || outputSpace.receiveOutputFromElevatorBottomLimitSwitch(ElevatorBottomLimitSwitchLocation.Values.PRESSED) == 0) {
            inputSpace.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.SET_SPEED, inputVal);
            inputSpace.sendInputToElevatorRightLift(ElevatorRightLiftMotorLocation.Action.SET_SPEED, -inputVal);
        }else{
            inputSpace.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.SET_SPEED, 0);
            inputSpace.sendInputToElevatorRightLift(ElevatorRightLiftMotorLocation.Action.SET_SPEED, 0);
        }

    }

    @Override
    public void stop() {
        inputSpace.stop();
        outputSpace.stop();
    }

}

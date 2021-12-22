package org.firstinspires.ftc.teamcode.main.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.main.utils.gamepads.GamepadManager;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardMotor;
import org.firstinspires.ftc.teamcode.main.utils.io.InputSpace;
import org.firstinspires.ftc.teamcode.main.utils.io.OutputSpace;
import org.firstinspires.ftc.teamcode.main.utils.locations.ElevatorBottomLimitSwitchLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.ElevatorLeftLiftMotorLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.ElevatorRightLiftMotorLocation;

@TeleOp(name="ElevatorTestTeleOp", group="physical_tests")
public class ElevatorTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadManager gamepadManager = new GamepadManager(gamepad1, gamepad1, gamepad1, gamepad1, gamepad1, gamepad1);
        InputSpace inputSpace = new InputSpace(hardwareMap);
        OutputSpace outputSpace = new OutputSpace(hardwareMap);
        waitForStart();
        resetStartTime();
        while(opModeIsActive()) {
            // elevator
            int elevatorInput = (gamepadManager.functionOneGamepad().right_bumper ? 0 : 1) + (gamepadManager.functionOneGamepad().left_bumper ? 0 : -1);
            int inputVal = ((StandardMotor) inputSpace.getElevatorLeftLift().getInternalInteractionSurface()).getDcMotor().getCurrentPosition() > -200 ? Range.clip(elevatorInput * 100, -100, 5) : Range.clip(elevatorInput * 100, -100, 100);
            if(inputVal < 0 || outputSpace.receiveOutputFromElevatorBottomLimitSwitch(ElevatorBottomLimitSwitchLocation.Values.PRESSED) == 0) {
                inputSpace.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.SET_SPEED, inputVal);
                inputSpace.sendInputToElevatorRightLift(ElevatorRightLiftMotorLocation.Action.SET_SPEED, -inputVal);
            }else{
                inputSpace.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.SET_SPEED, 0);
                inputSpace.sendInputToElevatorRightLift(ElevatorRightLiftMotorLocation.Action.SET_SPEED, 0);
            }
        }
        stop();
    }

}

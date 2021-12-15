package org.firstinspires.ftc.teamcode.competition.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.competition.utils.gamepads.GamepadManager;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardMotor;
import org.firstinspires.ftc.teamcode.competition.utils.io.InputSpace;
import org.firstinspires.ftc.teamcode.competition.utils.io.OutputSpace;
import org.firstinspires.ftc.teamcode.competition.utils.locations.ElevatorBottomLimitSwitchLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.ElevatorLeftLiftMotorLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.ElevatorRightLiftMotorLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.TankDrivetrainLocation;

@TeleOp(name="ElevatorTestTeleOp", group="physical_tests")
public class ElevatorTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadManager gamepads = new GamepadManager(gamepad1, gamepad1, gamepad1, gamepad1, gamepad1, gamepad1);
        InputSpace input = new InputSpace(hardwareMap);
        OutputSpace output = new OutputSpace(hardwareMap);
        waitForStart();
        resetStartTime();
        while(opModeIsActive()) {
            telemetry.addData("Tank? ", input.getTank().getInternalInteractionSurface());
            int inputVal = ((StandardMotor) input.getElevatorLeftLift().getInternalInteractionSurface()).getDcMotor().getCurrentPosition() > -200 ? (int) Range.clip(gamepads.functionOneGamepad().left_stick_y * 25, -5, 3) : (int) Range.clip(gamepads.functionOneGamepad().left_stick_y * 25, -25, 10);
            if(inputVal < 0 || output.receiveOutputFromElevatorBottomLimitSwitch(ElevatorBottomLimitSwitchLocation.Values.PRESSED) == 0) {
                input.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.SET_SPEED, inputVal);
                input.sendInputToElevatorRightLift(ElevatorRightLiftMotorLocation.Action.SET_SPEED, -inputVal);
            }else{
                input.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.SET_SPEED, 0);
                input.sendInputToElevatorRightLift(ElevatorRightLiftMotorLocation.Action.SET_SPEED, 0);
            }
        }
        stop();
    }

}

package org.firstinspires.ftc.teamcode.competition.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="ElevatorTestAutonomous", group="physical_tests")
public class ElevatorTestAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        InputSpace input = new InputSpace(hardwareMap);
        OutputSpace output = new OutputSpace(hardwareMap);
        waitForStart();
        resetStartTime();
        int inputVal = -6;
        input.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.MOVE_DISTANCE_IN_INCHES, inputVal);
        while(opModeIsActive()) {
            if(output.receiveOutputFromElevatorBottomLimitSwitch(ElevatorBottomLimitSwitchLocation.Values.PRESSED) == 1) {
                input.sendInputToElevatorLeftLift(ElevatorLeftLiftMotorLocation.Action.SET_SPEED, 0);
            }
        }
        stop();
    }

}

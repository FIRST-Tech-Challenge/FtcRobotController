package org.firstinspires.ftc.teamcode.main.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.utils.io.InputSpace;
import org.firstinspires.ftc.teamcode.main.utils.io.OutputSpace;
import org.firstinspires.ftc.teamcode.main.utils.locations.ElevatorBottomLimitSwitchLocation;
import org.firstinspires.ftc.teamcode.main.utils.locations.ElevatorLeftLiftMotorLocation;

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

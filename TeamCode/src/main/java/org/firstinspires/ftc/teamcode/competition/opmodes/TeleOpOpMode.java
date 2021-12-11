package org.firstinspires.ftc.teamcode.competition.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.competition.utils.gamepads.GamepadManager;
import org.firstinspires.ftc.teamcode.competition.utils.io.InputSpace;
import org.firstinspires.ftc.teamcode.competition.utils.locations.TankDrivetrainLocation;

@TeleOp(name="I_AM_THE_WORKING_ONE_TankTeleOp", group="linear")
public class TeleOpOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadManager gamepads = new GamepadManager(gamepad1, gamepad1, gamepad1, gamepad1, gamepad1, gamepad1);
        InputSpace input = new InputSpace(hardwareMap);
        waitForStart();
        resetStartTime();
        while(opModeIsActive()) {
            telemetry.addData("Tank? ", input.getTank().getInternalInteractionSurface());
            int right = (int) Range.clip(gamepads.functionOneGamepad().left_stick_y * 25, -25, 25);
            int left = (int) Range.clip(gamepads.functionOneGamepad().right_stick_y * 25, -25, 25);
            input.sendInputToTank(TankDrivetrainLocation.Action.SET_SPEED, right, left);
        }
        stop();
    }

}

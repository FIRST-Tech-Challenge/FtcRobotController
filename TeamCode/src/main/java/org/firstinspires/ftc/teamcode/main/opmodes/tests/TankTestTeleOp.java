package org.firstinspires.ftc.teamcode.main.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.main.utils.gamepads.GamepadManager;
import org.firstinspires.ftc.teamcode.main.utils.io.InputSpace;
import org.firstinspires.ftc.teamcode.main.utils.io.OutputSpace;
import org.firstinspires.ftc.teamcode.main.utils.locations.TankDrivetrainLocation;

@TeleOp(name="TankTestTeleOp", group="physical_tests")
public class TankTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadManager gamepads = new GamepadManager(gamepad1, gamepad1, gamepad1, gamepad1, gamepad1, gamepad1);
        InputSpace input = new InputSpace(hardwareMap);
        OutputSpace output = new OutputSpace(hardwareMap);
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

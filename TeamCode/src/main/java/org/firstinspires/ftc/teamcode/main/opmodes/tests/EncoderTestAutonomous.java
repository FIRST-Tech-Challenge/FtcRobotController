package org.firstinspires.ftc.teamcode.main.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.main.utils.gamepads.GamepadManager;
import org.firstinspires.ftc.teamcode.main.utils.interactions.groups.StandardDrivetrain;
import org.firstinspires.ftc.teamcode.main.utils.interactions.groups.StandardVehicleDrivetrain;
import org.firstinspires.ftc.teamcode.main.utils.io.InputSpace;
import org.firstinspires.ftc.teamcode.main.utils.io.OutputSpace;
import org.firstinspires.ftc.teamcode.main.utils.locations.TankDrivetrainLocation;
import org.firstinspires.ftc.teamcode.main.utils.scripting.AutonomousStubScript;
import org.firstinspires.ftc.teamcode.main.utils.scripting.Script;
import org.firstinspires.ftc.teamcode.main.utils.scripting.ScriptRunner;

@Autonomous(name="Encoder Test", group="physical_tests") // replace the name and group with your OpMode's name and group
public class EncoderTestAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadManager gamepads = new GamepadManager(gamepad1, gamepad1, gamepad1, gamepad1, gamepad1, gamepad1);
        InputSpace input = new InputSpace(hardwareMap);
        OutputSpace output = new OutputSpace(hardwareMap);

        StandardVehicleDrivetrain standardDrivetrain = (StandardVehicleDrivetrain) input.getTank().getInternalInteractionSurface();

        waitForStart();
        resetStartTime();

        while (opModeIsActive()) {
            standardDrivetrain.driveDistance(12, 100);
        }
        stop();
    }

}

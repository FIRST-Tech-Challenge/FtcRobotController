package org.firstinspires.ftc.teamcode.opModes.team2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.joystickMappings.CosMapping;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.joystickMappings.RootMapping;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.subsystems.drivetrain.controllers.DriveTrainController;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.Maths;

@TeleOp(name = "Team 2 - TeleOp", group = "2_TeleOp")
public class TeleOpTest extends OpMode {

    private DriveTrainController driveTrain;

    private DiscretePositionArm arm; // TODO


    DebouncedButton raiseArmButton; // TODO
    DebouncedButton floatArmButton;
    DebouncedButton powerDownArmButton;

    @Override
    public void init() {
        raiseArmButton = new DebouncedButton(GamepadButton.TRIANGLE); // TODO
        floatArmButton = new DebouncedButton(GamepadButton.CIRCLE);
        powerDownArmButton = new DebouncedButton(GamepadButton.CROSS);
        driveTrain = new DriveTrainController(new DriveTrain( // TODO
                hardwareMap.get(DcMotor.class, "left_drivetrain_motor"),
                hardwareMap.get(DcMotor.class, "right_drivetrain_motor"),
                false
        ),
                new RootMapping(2),
                new CosMapping(),
                Constants.TEAM2_DRIVETRAIN_FORWARDS_GRADIENT, // TODO
                Constants.TEAM2_DRIVETRAIN_FORWARDS_INTERCEPT
        );
        arm = new DiscretePositionArm(
                hardwareMap.get(DcMotor.class, "slide"),
                false,
                Constants.TEAM2_SLIDE_FRONT_COUNTS,
                Constants.TEAM2_SLIDE_BACK_COUNTS
        );

    }

    @Override
    public void start() {
        arm.moveToFront(Constants.TEAM2_SLIDE_SPEED); // Hover
    }

    @Override
    public void loop() {

            /* Drivetrain */
            // CONTROLS: Left joystick
            XY leftJoystick = Inputs.getLeftJoystickData(); // TODO
            XY rightJoystick = Inputs.getRightJoystickData(); // TODO

            double speed = Maths.clamp(-leftJoystick.y - rightJoystick.y, -1, 1);
            double turn = Maths.clamp(-leftJoystick.x + rightJoystick.x, -1, 1);

            driveTrain.drive_scaled(speed, turn, scale, 0.4); // TODO

            telemetry.update();

    }
}

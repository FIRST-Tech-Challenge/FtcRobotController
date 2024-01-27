package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "servo test")
public class TestActuatorAuton extends CommandOpMode {
    private SimpleServo left_claw, right_claw;
    private GamepadEx gpad;
    @Override
    public void initialize() {
        left_claw = new SimpleServo(hardwareMap, "lc", -180, 180);
        right_claw = new SimpleServo(hardwareMap, "rc", -180, 180);

        schedule(
            new WaitUntilCommand(this::isStarted)
                .andThen(new SequentialCommandGroup(
                        new InstantCommand(() -> left_claw.turnToAngle(-115)).andThen(new InstantCommand(() -> right_claw.turnToAngle(30))),
                        new InstantCommand(() -> left_claw.turnToAngle(100)).andThen(new InstantCommand(() -> right_claw.turnToAngle(-70)))
                    )));
    }
}

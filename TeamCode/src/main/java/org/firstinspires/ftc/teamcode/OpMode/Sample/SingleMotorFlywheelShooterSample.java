package org.firstinspires.ftc.teamcode.OpMode.Sample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.SingleMotorFlywheelShooter;

@Disabled
@TeleOp()
public class SingleMotorFlywheelShooterSample extends OpMode {

    private SingleMotorFlywheelShooter shooter;

    @Override
    public void init() {
        shooter = new SingleMotorFlywheelShooter(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            shooter.shoot();
        }

        if (gamepad1.bWasPressed()) {
            shooter.shoot();
        }

        if (gamepad1.xWasPressed()) {
            shooter.idle();
        }

        shooter.shooterStateMachine();
    }
}

package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemFF;


@TeleOp(name = "MainTeleOp- light testing")
public class lighttesting extends LinearOpMode {
    private LightSubsystem lights;
    private ShooterSubsystemFF shooter;

    @Override
    public void runOpMode() {
        lights = new LightSubsystem(hardwareMap);
        shooter = new ShooterSubsystemFF(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // Update the light
            //lights.update(shooter.isOn(), shooter.isReady());
            lights.setGreen();
        }

    }
}
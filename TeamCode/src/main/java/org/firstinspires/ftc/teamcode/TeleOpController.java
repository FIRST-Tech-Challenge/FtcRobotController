package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Field-Centric TeleOp")
public class TeleOpController extends OpMode {
    TeleOpDriver driveTrain = new TeleOpDriver();

    Boolean LastCheck = Boolean.FALSE;
    Boolean Current = Boolean.FALSE;

    @Override
    public void init() {
        driveTrain.intialiseDriver();
    }

    @Override
    public void loop() {
        // Default cap of 0.75 power normally, removed (1.0) while R2 (right_trigger) is held down
        double speedCap = 0.75;
        if (gamepad1.left_trigger > 0.5) {
            speedCap = 1.0;
        }

        // Reset heading check
        Current = gamepad1.y;
        if (Current & !LastCheck) {
            driveTrain.resetHeading();
        }
        LastCheck = Current;

        // Run drive loop with the dynamic speed multiplier
        driveTrain.DriveLoop(speedCap);
    }
}

/*
* Controls:
* Y button - zero Field-centric drive direction and coordinates
* L2 Held more than half-way - removes 0.75 "precision" speed cap and allows you to reach full speed
* R2 (Reserved not implemented yet) - Shoot Balls
* R1 (Reserved not implemented yet) - Toggle on Intake
* Left Joystick Y - Forwards and backwards movement
* Left Joystick X - Left and Right Strafing
* Right Joystick X - Rotation Clockwise and Counter-Clockwise
* L1 (Planned not implemented yet) - Hold Break Toggle
* 
* */

package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.components.LiftComponent;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

public class TestLiftComponent extends TeleOpModeBase {

    LiftComponent lift;

    @Override
    public void setup() {
        Motor lift_motor = HardwareMapContainer.motor0;
        lift = new LiftComponent(lift_motor, 0.42, (int)((288 / 3) / (Math.PI*2)), 0); // Core Hex Motor has 288 counts/revolution; counts/radian = counts/revn / (radians/revn); 3:1 gear
    }

    @Override
    public void every_tick() {
        // TODO: 21/01/2023 Add controls 
    }
}

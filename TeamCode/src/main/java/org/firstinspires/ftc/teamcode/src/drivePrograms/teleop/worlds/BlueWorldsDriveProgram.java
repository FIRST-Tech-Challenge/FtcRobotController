package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.worlds;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "🟦Blue Worlds Drive Program🟦")
public class BlueWorldsDriveProgram extends RedWorldsDriveProgram {
    public BlueWorldsDriveProgram(){
        super();
        this.defaultColor = BlinkinPattern.BLUE;
        this.currentPattern = this.defaultColor;
    }
}

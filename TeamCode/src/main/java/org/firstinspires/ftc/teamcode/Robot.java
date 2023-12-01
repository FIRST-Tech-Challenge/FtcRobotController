// This file is a system file.
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public DriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;

    // This combines all the subsystems.
    public Robot(HardwareMap hwMap) {
        driveTrain = new DriveTrain(hwMap);
        //intake = new Intake(hwMap);
       // outtake = new Outtake(hwMap);
    }

}
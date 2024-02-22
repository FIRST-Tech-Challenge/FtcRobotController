// This file is a system file.
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gobbler {
    public DriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;
    public PlaneHang planeHang;

    // This combines all the subsystems.
    public Gobbler(HardwareMap hwMap) {
        driveTrain = new DriveTrain(hwMap);
        intake = new Intake(hwMap);
        outtake = new Outtake(hwMap);
        planeHang = new PlaneHang(hwMap);
    }
}
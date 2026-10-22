package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
     RobotHardware hardware;

     public Robot(HardwareMap hw) {
        hardware = new RobotHardware(hw);
     }

     public void runDriveTrain(DriveValues vals) {
         hardware.leftDriveFront.setPower(vals.FL);
         hardware.rightDriveFront.setPower(vals.FR);
         hardware.leftDriveBack.setPower(vals.BL);
         hardware.rightDriveBack.setPower(vals.BR);
     }

     public void endDriveTrain() {
         hardware.leftDriveFront.setPower(0);
         hardware.rightDriveFront.setPower(0);
         hardware.leftDriveBack.setPower(0);
         hardware.rightDriveBack.setPower(0);
     }
}

/*
 * Base code to manage the robot. Baseline copied from FtcRobotController sample code
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*Comment
 armManager.SetDirection(AutoDriveManager.DriveDirection.BACKWARD);
                armManager.MoveArmToPosition(AutoArmManager.ARM_MIN);
 */
public class ArmManager {

    public static final double ARM_SPEED = 0.05;
    public static final double ARM_MIN = 0.10 ;
    public static final double ARM_MAX = 0.60;

    /* Declare OpMode members. */
    private LinearOpMode opMode = null;   // gain access to methods in the calling OpMode.

    private HornetRobo hornetRobo;

    public ArmManager(LinearOpMode LOpMode, HornetRobo HRobo) {

        opMode = LOpMode;
        hornetRobo = HRobo;
    }

    public void SetDirection(DriveManager.DriveDirection Direction)
    {
        if (Direction == DriveManager.DriveDirection.FORWARD) {
            hornetRobo.ArmOne.setDirection(Servo.Direction.FORWARD);
            hornetRobo.ArmTwo.setDirection(Servo.Direction.REVERSE);
        }
        else if (Direction == DriveManager.DriveDirection.BACKWARD) {
            hornetRobo.ArmOne.setDirection(Servo.Direction.REVERSE);
            hornetRobo.ArmTwo.setDirection(Servo.Direction.FORWARD);
        }
    }

    public void MoveArmToPosition(double Position){
        hornetRobo.ArmOne.setPosition(Range.clip(Position, ARM_MIN, ARM_MAX));
        hornetRobo.ArmTwo.setPosition(Range.clip(Position, ARM_MIN, ARM_MAX));
    }

    public void MoveArmToPosition(double Position, double ArmMin, double ArmMax){
        hornetRobo.ArmOne.setPosition(Range.clip(Position, ArmMin, ArmMax));
        hornetRobo.ArmTwo.setPosition(Range.clip(Position, ArmMin, ArmMax));
    }
    public double GetArmServoPosition(int armNumber){
        if (armNumber == 1)
            return hornetRobo.ArmOne.getPosition();
        return hornetRobo.ArmTwo.getPosition();
    }

}



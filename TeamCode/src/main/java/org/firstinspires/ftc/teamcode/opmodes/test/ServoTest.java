package org.firstinspires.ftc.teamcode.opmodes.test;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest extends BlocksOpModeCompanion {

    @ExportToBlocks (
            comment = "Move a conventional servo back and forth. Assumes servo starts" +
                    " from position 0. Servo name must be in the active configuration.",
            tooltip = "Wiggle a user-designated servo.",
            parameterLabels = {"Servo name", "Duration (milliseconds)", "Number of cycles"}
    )
    public static void wiggleServo (String servoName, int duration, int cycles) {

        Servo myServo = hardwareMap.get(Servo.class, servoName);

        // count up to 'cycles' AND while opMode was not stopped
        for (int i = 0; i < cycles && linearOpMode.opModeIsActive(); i++)  {

            myServo.setPosition(0.5);              // move servo clockwise
            linearOpMode.sleep(duration);          // wait for 'duration'
            myServo.setPosition(0);                // move servo counterclockwise
            linearOpMode.sleep(duration);          // wait for 'duration'
        }
    }   // end method wiggleServo()
}       // end class SampleMyBlocks_v01
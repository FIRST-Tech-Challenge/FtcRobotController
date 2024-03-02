package org.firstinspires.ftc.teamcode.Testing.AutonRoutes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.Testing.Auton_test.AutonDriveToAprilTagAdisVersion;
import org.firstinspires.ftc.teamcode.Testing.Helper_test.AdisEncoderDrive;

//This program was made by Adi. It is meant to be where we can acess documentation about the encoder drive functions
@Autonomous(name="A2_S_B1_Test", group = "tool")
@Disabled
public class A2_S_B1_Test extends LinearOpMode
{
    // Adjust these numbers to suit your robot.


    @Override
    public void runOpMode()
    {
        AdisEncoderDrive adisEncoderDrive = new AdisEncoderDrive();
        adisEncoderDrive.prepareEncoder();

        waitForStart();

        adisEncoderDrive.encoderDriveForwardInches(2.0);
        adisEncoderDrive.encoderDriveLeftBlocks(3.2);
        adisEncoderDrive.encoderDriveForwardBlocks(0.8);
        adisEncoderDrive.TurnLeft(105);
        adisEncoderDrive.encoderDriveForwardInches(7.2);


    }


}

package org.firstinspires.ftc.teamcode.competition.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.other.utils.Motor;

@Autonomous(name="Encoder Drive Test (H-Drive)", group="H.Testing.Autonomous")
public class AutoEncoderDriveTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        Motor left = new Motor(telemetry, hardwareMap, "ld1", DcMotorSimple.Direction.FORWARD, 400, 2.0, 2.0);
        left.driveDistance(60, 50);

        while(left.getDcMotor().isBusy());
    }

}

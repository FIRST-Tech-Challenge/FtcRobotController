package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.utils.motors.ComplexMotorController;
import org.firstinspires.ftc.teamcode.utils.motors.Motor;
import org.firstinspires.ftc.teamcode.utils.motors.SimpleMotorController;

@Autonomous(name="Encoder Drive Test (H-Drive)", group="H.Testing.Autonomous")
public class AutoEncoderDriveTest extends LinearOpMode {

    @Override
    public void runOpMode() {


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        Motor flipper = new Motor(telemetry, hardwareMap, "flipper", DcMotorSimple.Direction.FORWARD, 1400, 2.0, 2.0);
        ComplexMotorController complexMotorController = new ComplexMotorController(telemetry);
        complexMotorController.prepMotor(flipper, 10);
        complexMotorController.startMotor(flipper, 1);
        while(flipper.getDcMotor().isBusy()){}
        complexMotorController.stopMotor(flipper);
    }

}

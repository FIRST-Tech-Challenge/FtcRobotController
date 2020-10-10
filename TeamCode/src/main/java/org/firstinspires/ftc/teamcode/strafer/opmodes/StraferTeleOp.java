package org.firstinspires.ftc.teamcode.strafer.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technototes.library.control.gamepad.CommandGamepad;
import com.technototes.library.structure.TeleOpCommandOpMode;
import com.technototes.logger.Loggable;
import com.technototes.logger.Logger;

import org.firstinspires.ftc.teamcode.strafer.OI;
import org.firstinspires.ftc.teamcode.strafer.Robot;


@TeleOp(name = "Strafer TeleOp2")
public class StraferTeleOp extends TeleOpCommandOpMode implements Loggable {
    public OI oi;

    public Robot robot;

    //test change

    @Override
    public void beginInit() {
        driverGamepad = new CommandGamepad(gamepad1);
        codriverGamepad = new CommandGamepad(gamepad2);
        robot = new Robot(hardwareMap, telemetry);
        oi = new OI(driverGamepad, codriverGamepad, robot);
    }

    @Override
    public void runLoop() {
        robot.drivebaseSubsystem.joystickDriveWithGyro(driverGamepad.leftStick.getYAxis(), driverGamepad.leftStick.getXAxis(), driverGamepad.rightStick.getYAxis(), robot.hardware.imu.gyroHeading());

    }
}

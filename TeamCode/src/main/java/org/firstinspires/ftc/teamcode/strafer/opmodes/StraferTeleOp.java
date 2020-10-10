package org.firstinspires.ftc.teamcode.strafer.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technototes.library.control.gamepad.CommandGamepad;
import com.technototes.library.structure.CommandOpMode;
import com.technototes.library.structure.TeleOpCommandOpMode;
import com.technototes.logger.Loggable;
import com.technototes.logger.Logger;

import org.firstinspires.ftc.teamcode.strafer.OI;
import org.firstinspires.ftc.teamcode.strafer.Robot;


@TeleOp(name = "Strafer TeleOp2")
public class StraferTeleOp extends CommandOpMode implements Loggable {
    public OI oi;

    public Robot robot;

    //test change

    @Override
    public void beginInit() {
        robot = new Robot(hardwareMap, telemetry);
        oi = new OI(driverGamepad, codriverGamepad, robot);
    }

    @Override
    public void runLoop() {
        robot.drivebaseSubsystem.joystickDriveWithGyro(driverGamepad.leftStick.getXAxis(), driverGamepad.leftStick.getYAxis(), driverGamepad.rightStick.getXAxis(), robot.hardware.imu.gyroHeading());

    }
}

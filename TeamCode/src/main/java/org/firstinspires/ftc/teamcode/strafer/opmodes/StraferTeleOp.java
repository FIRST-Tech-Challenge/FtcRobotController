package org.firstinspires.ftc.teamcode.strafer.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technototes.library.structure.CommandOpMode;
import com.technototes.logger.Loggable;

import org.firstinspires.ftc.teamcode.strafer.OperatorInterface;
import org.firstinspires.ftc.teamcode.strafer.Robot;


@TeleOp(name = "Strafer TeleOp2")
public class StraferTeleOp extends CommandOpMode implements Loggable {
    public OperatorInterface oi;

    public Robot robot;

    //test change

    @Override
    public void uponInit() {
        robot = new Robot();
        oi = new OperatorInterface(driverGamepad, codriverGamepad, robot);
    }

    @Override
    public void runLoop() {
        robot.drivebaseSubsystem.joystickDriveWithGyro(driverGamepad.leftStick.getXAxis(), driverGamepad.leftStick.getYAxis(), driverGamepad.rightStick.getXAxis(), robot.hardware.imu.gyroHeading());
    }

}

package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Blinkin;
import org.firstinspires.ftc.teamcode.utility.BlinkinColour;

import java.lang.annotation.ElementType;
import java.lang.annotation.Target;


// command template
public class MoveToPickup extends CommandBase {
    PIDController xControl;
    PIDController yControl;
    double xError;
    double yError;
    ElapsedTime timer;
    // constructor
    public MoveToPickup() {
        xControl = new PIDController(0.0032, 0, 0.000245);
        yControl = new PIDController(0.0032, 0, 0.000245);

        timer = new ElapsedTime();
        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        xControl.reset();
        yControl.reset();

        timer.reset();

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        double Target_X = 296;
        double Target_Y = 316;
        xError = Target_X - RobotContainer.piece_center_X;
        yError = Target_Y - RobotContainer.piece_center_Y;

        double xSpeed = -xControl.calculate(xError);
        double ySpeed = -yControl.calculate(yError);

        RobotContainer.drivesystem.RobotDrive(ySpeed, xSpeed, 0);

        if (xError>20 || yError>20) {
            timer.reset();
        }
        RobotContainer.DBTelemetry.addData("xSpeed", xSpeed);
        RobotContainer.DBTelemetry.addData("ySpeed", ySpeed);
        RobotContainer.DBTelemetry.update();
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        if (timer.seconds()>1.5) {
            RobotContainer.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
            return true;
        }else {
            return false;
        }

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivesystem.RobotDrive(0,0,0);
    }

}
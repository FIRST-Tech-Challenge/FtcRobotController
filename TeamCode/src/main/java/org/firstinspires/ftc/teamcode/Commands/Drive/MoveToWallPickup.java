package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utility.VisionProcessorMode;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import java.util.List;


// command template
public class MoveToWallPickup extends CommandBase {

    // controller to steer towards item
    PIDController xControl;
    PIDController omegaControl;

    // constructor
    public MoveToWallPickup() {

        xControl = new PIDController(0.0025, 0.0, 0.0);
        omegaControl = new PIDController(0.1, 0.0005, 0.0);

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // initialize y controller
        xControl.reset();

        // set camera for pickup
        if (RobotContainer.isRedAlliance())
            RobotContainer.clawCamera.setVisionProcessingMode(VisionProcessorMode.RED_WALLPICKUP_BLOB);
        else
            RobotContainer.clawCamera.setVisionProcessingMode(VisionProcessorMode.BLUE_WALLPICKUP_BLOB);

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

        // speed to move forward at
        double x_speed;
        double y_speed;
        double omega_speed;

        // get list of blob detections from camera
        List<ColorBlobLocatorProcessor.Blob> detections = RobotContainer.clawCamera.GetBlobDetections();

        // do we have valid detections
        if (detections!=null && !detections.isEmpty())
        {
            double piece_center_X = detections.get(0).getBoxFit().center.x;

            // determine sideways speed
            x_speed = xControl.calculate(320.0 - piece_center_X);
        }
        else
            // no detection, do not move sideways
            x_speed = 0.0;


        // determine forward speed (m/s)
        y_speed = 0.30;

        // if on red alliance, field directions are opposite
        if (RobotContainer.isRedAlliance())
        {
            x_speed *=-1.0;
            y_speed *=-1.0;
        }

        // rotational control
        double targetangle = -90.0;
        if (RobotContainer.isRedAlliance())
            targetangle = +90.0;
        double currentAngle = RobotContainer.odometry.getCurrentPos().getRotation().getDegrees();
        omega_speed = -omegaControl.calculate(targetangle - currentAngle);

        // drive robot
        RobotContainer.drivesystem.FieldDrive(x_speed, y_speed, omega_speed);

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        return RobotContainer.clawTouch.ControlTouchSenser();
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        // turn off drive
        RobotContainer.drivesystem.FieldDrive(0,0,0);

        // turn off camera
        RobotContainer.clawCamera.setVisionProcessingMode(VisionProcessorMode.NONE);
    }

}
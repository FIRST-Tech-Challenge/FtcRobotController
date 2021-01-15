package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.HzDriveConstantsDriveEncoders;
import org.firstinspires.ftc.teamcode.drive.HzMecanumDriveDriveEncoders;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.DriveConstantsDeadWheelEncoder;
//import org.firstinspires.ftc.teamcode.drive.MecanumDriveDeadWheelsEncoder;

//public class HzDrive extends SampleMecanumDrive {
public class HzDrive extends HzMecanumDriveDriveEncoders {
    //double DriveConstants_kV = DriveConstants.kV;
    //double DriveConstants_kV = DriveConstantsDeadWheelEncoder.kV;
    double DriveConstants_kV = HzDriveConstantsDriveEncoders.kV;

    //double DriveConstants_TRACK_WIDTH = DriveConstants.TRACK_WIDTH;
    //double DriveConstants_TRACK_WIDTH = DriveConstantsDeadWheelEncoder.TRACK_WIDTH;
    double DriveConstants_TRACK_WIDTH = HzDriveConstantsDriveEncoders.TRACK_WIDTH;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    //private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    //private PIDFController headingController = new PIDFController(MecanumDriveDeadWheelsEncoder.HEADING_PID);
    private PIDFController headingController = new PIDFController(HzMecanumDriveDriveEncoders.HEADING_PID);

    HzGameField hzGameField;

    enum DriveType {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
    }

    public DriveType driveType = DriveType.FIELD_CENTRIC;

    //**** Align to point and Field Drive Mode ****
    // Define 2 states, driver control or alignment control
    enum DriveMode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT;
        DriveMode toggle() {
            if (this.equals(NORMAL_CONTROL))
                return ALIGN_TO_POINT;
            else
                return NORMAL_CONTROL;
        }
    }

    enum AugmentedControl {
        NONE,
        TURN_CENTER,
        TURN_DELTA_LEFT,
        TURN_DELTA_RIGHT
    }

    public AugmentedControl augmentedControl = AugmentedControl.NONE;

    public DriveMode driveMode = DriveMode.NORMAL_CONTROL; //Default initializer
    public static double DRAWING_TARGET_RADIUS = 2;
    public Vector2d drivePointToAlign = hzGameField.ORIGIN;


    //**** Drive Train ****
    //For Position
    public Pose2d poseEstimate = new Pose2d(0,0,0);

    public HzDrive(HardwareMap hardwareMap) {
        super(hardwareMap);

    }

    public Vector2d gamepadInput = new Vector2d(0,0);
    public double gamepadInputTurn = 0;

    public void driveTrainFieldCentric(){

        //poseEstimate = getPoseEstimate();

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        setWeightedDrivePower(
                new Pose2d(
                        gamepadInput.getX(),
                        gamepadInput.getY(),
                        gamepadInputTurn
                )
        );

        // Update everything. Odometry. Etc.
        update();
    }



    public void driveTrainPointFieldModes(){
        //poseEstimate = getPoseEstimate();
        //TODO : TESTING VUFORIA RUNNING IN PARALLEL

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

        // Declare a drive direction
        // Pose representing desired x, y, and angular velocity
        Pose2d driveDirection = new Pose2d();

        //callingOpMode.telemetry.addData("mode", driveMode);

        // Declare telemetry packet for dashboard field drawing
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        switch (driveMode) {
            case NORMAL_CONTROL:
                driveDirection = new Pose2d(
                        gamepadInput.getX(),
                        gamepadInput.getY(),
                        gamepadInputTurn
                );

                switch (augmentedControl){
                    case NONE :
                        augmentedControl = AugmentedControl.NONE;
                        break;
                    case TURN_CENTER:
                        turnAsync(Angle.normDelta(Math.toRadians(0) - poseEstimate.getHeading()));
                        augmentedControl = AugmentedControl.NONE;
                        break;
                    case TURN_DELTA_LEFT:
                        turnAsync(Angle.normDelta(Math.toRadians(-5)));
                        augmentedControl = AugmentedControl.NONE;
                        break;
                    case TURN_DELTA_RIGHT:
                        turnAsync(Angle.normDelta(Math.toRadians(5)));
                        augmentedControl = AugmentedControl.NONE;
                        break;
                }
                break;

            case ALIGN_TO_POINT:
                // Create a vector from the gamepad x/y inputs which is the field relative movement
                // Then, rotate that vector by the inverse of that heading for field centric control
                Vector2d fieldFrameInput = new Vector2d(
                        gamepadInput.getX(),
                        gamepadInput.getY()
                );
                Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                // Difference between the target vector and the bot's position
                Vector2d difference = drivePointToAlign.minus(poseEstimate.vec());
                // Obtain the target angle for feedback and derivative for feedforward
                double theta = difference.angle();

                // Not technically omega because its power. This is the derivative of atan2
                double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                // Set the target heading for the heading controller to our desired angle
                headingController.setTargetPosition(theta);

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(poseEstimate.getHeading())
                        //* DriveConstants.kV + thetaFF)
                        //* DriveConstants.TRACK_WIDTH;
                        * DriveConstants_kV + thetaFF)
                        * DriveConstants_TRACK_WIDTH;

                // Combine the field centric x/y velocity with our derived angular velocity
                driveDirection = new Pose2d(
                        robotFrameInput,
                        headingInput
                );

                // Draw the target on the field
                fieldOverlay.setStroke("#dd2c00");
                fieldOverlay.strokeCircle(drivePointToAlign.getX(), drivePointToAlign.getY(), DRAWING_TARGET_RADIUS);

                // Draw lines to target
                fieldOverlay.setStroke("#b89eff");
                fieldOverlay.strokeLine(drivePointToAlign.getX(), drivePointToAlign.getY(), poseEstimate.getX(), poseEstimate.getY());
                fieldOverlay.setStroke("#ffce7a");
                fieldOverlay.strokeLine(drivePointToAlign.getX(), drivePointToAlign.getY(), drivePointToAlign.getX(), poseEstimate.getY());
                fieldOverlay.strokeLine(drivePointToAlign.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());
                break;
        }

        // Draw bot on canvas
        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

        setWeightedDrivePower(driveDirection);

        // Update the heading controller with our current heading
        headingController.update(poseEstimate.getHeading());

        // Update the localizer
        getLocalizer().update();

        //TODO : TRY UPDATING TO Vuforia POS estimate here.



        // Send telemetry packet off to dashboard
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }


}

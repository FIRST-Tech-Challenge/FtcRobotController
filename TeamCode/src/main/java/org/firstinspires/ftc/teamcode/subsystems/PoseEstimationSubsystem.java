package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Base64;

public class PoseEstimationSubsystem extends SubsystemBase {

    HardwareMap m_hardwareMap;
    private IMU m_imu;
    Telemetry m_telemetry;
    DcMotorEx parallelEncoder;
    DcMotorEx prependicularEncoder;

    public static double x = 0.0;                // Initial x-position
    public static double y = 0.0;                // Initial y-position
    public static double heading = 0.0;          // Initial orientation (in radians)
    public static double prevParallelEncoderPos = 0.0;         // Previous parallel encoder count
    public static double prevPerpendicularEncoderPos = 0.0;     // Previous perpendicular encoder count
    public static double prevImuHeading = 0.0;   // Previous IMU heading

    // Wheelbase distance (distance between parallel and perpendicular encoders)
    private double wheelbase = 0.2;  // Replace with the actual value
    public static double deltaParallelEncoderPos = 0;
    public static double deltaPerpendicularEncoderPos = 0;

    public static double prependicularEncoderPosition = 0;
    public static double parallelEncoderPosition = 0;

    public static double offsetParallel = 1.5;
    public static double offsetPrependicular = 2;


    public PoseEstimationSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_imu = m_hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,

                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                );
        m_imu.initialize(new IMU.Parameters(orientationOnRobot));


        parallelEncoder = m_hardwareMap.get(DcMotorEx.class, "pe");
        prependicularEncoder = m_hardwareMap.get(DcMotorEx.class, "ppe");

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        prependicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void reset(double Px, double Py){
         x = Px;                // Initial x-position
         y = Py;                // Initial y-position
         heading = 0.0;          // Initial orientation (in radians)
         prevParallelEncoderPos = 0.0;         // Previous parallel encoder count
         prevPerpendicularEncoderPos = 0.0;     // Previous perpendicular encoder count
         prevImuHeading = 0.0;   // Previous IMU heading

        // Wheelbase distance (distance betw
        deltaParallelEncoderPos = 0;
        deltaPerpendicularEncoderPos = 0;

        prependicularEncoderPosition = 0;
        parallelEncoderPosition = 0;


    }

    public Pose2d positionEstimation(){
        double headingOffset = m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        prependicularEncoderPosition = prependicularEncoder.getCurrentPosition();
        parallelEncoderPosition = parallelEncoder.getCurrentPosition();


        deltaParallelEncoderPos = parallelEncoderPosition - prevParallelEncoderPos;
        deltaPerpendicularEncoderPos = prependicularEncoderPosition - prevPerpendicularEncoderPos;

        // Calculate the change in orientation
        // Update previous values for the next iteration
        prevParallelEncoderPos = parallelEncoderPosition;
        prevPerpendicularEncoderPos = prependicularEncoderPosition;
        prevImuHeading = m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;

        // Update the position estimate
        x += deltaParallelEncoderPos * Math.cos(heading) - deltaPerpendicularEncoderPos * Math.sin(heading);
        y += deltaParallelEncoderPos * Math.sin(heading) + deltaPerpendicularEncoderPos * Math.cos(heading);


        // Update the heading
        heading = m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) -headingOffset;

        return new Pose2d(new Translation2d(x/2000 * 1.88976,y/2000 * 1.88976),new Rotation2d(heading));

    }


    public double getHeadingAsDouble(Pose2d positionEstimation){
        return positionEstimation.getHeading();
    }
    public void resetHeading(){m_imu.resetYaw();}

    public Command resetHeadingCommand(){
        return new InstantCommand(() -> {resetHeading();});

    }

    public Pose2d getPose()
    {
        double heading = m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return new Pose2d(new Translation2d(0.0,0.0),new Rotation2d(heading));
    }

    @Override
    public void periodic()
    {
        positionEstimation();
        m_telemetry.addData("RobotAngle", getPose().getHeading());
           m_telemetry.addData("RobotPose", positionEstimation());

        m_telemetry.addData("Encoder", prependicularEncoder.getCurrentPosition());
        m_telemetry.addData("Encoder", prependicularEncoder.getCurrentPosition());
        m_telemetry.update();
    }
}

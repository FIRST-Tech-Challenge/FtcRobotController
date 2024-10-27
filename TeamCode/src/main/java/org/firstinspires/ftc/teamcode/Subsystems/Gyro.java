package org.firstinspires.ftc.teamcode.Subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;

/** Gyro Subsystem */
public class Gyro extends SubsystemBase {

    // robot gyroscope
    private IMU gyro;

    // gyro offset
    private double YawAngleOffset;

    /** Place code here to initialize subsystem */
    public Gyro() {
        // create gyro and initialize it
        YawAngleOffset = 0.0;
        gyro = RobotContainer.ActiveOpMode.hardwareMap.get(IMU.class, "imu");
        gyro.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        gyro.resetYaw();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        //RobotContainer.ActiveOpMode.telemetry.addData("Gyro", JavaUtil.formatNumber(getYawAngle(), 2));
    }

    /** get gyro angle - returns angle in deg between -180 and 180 */
    public double getYawAngle() {

        // positive for 'Tiny' robot
        return (gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))+YawAngleOffset;
    }

    // resets gyro and offset value
    public void resetYawAngle() {
        setYawAngle(0.0);
    }

    // sets gyro to provided angle (in deg)
    public void setYawAngle(double angle) {

        YawAngleOffset -= getYawAngle() - angle;
    }

}

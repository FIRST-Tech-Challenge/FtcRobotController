package com.technototes.library.command.simple;

import com.technototes.control.gamepad.Stick;
import com.technototes.library.command.Command;
import com.technototes.library.hardware.sensor.IMU;
import com.technototes.library.subsystem.drivebase.MecanumDrivebaseSubsystem;

import java.util.function.DoubleSupplier;

/** Simple command to drive mecanum robot
 * @author Alex Stedman
 */
public class MecanumDriveCommand extends Command {
    private MecanumDrivebaseSubsystem subsystem;
    private DoubleSupplier xv, yv, tv;
    private DoubleSupplier gyro;

    /** Make mecanum drive command
     *
     * @param subsystem The {@link MecanumDrivebaseSubsystem} for this command
     * @param xSupplier The supplier for x movement
     * @param ySupplier The supplier for y movement
     * @param twistSupplier The supplier for robot twist
     */
    public MecanumDriveCommand(MecanumDrivebaseSubsystem subsystem, DoubleSupplier xSupplier,
                               DoubleSupplier ySupplier, DoubleSupplier twistSupplier){
        addRequirements(subsystem);
        this.subsystem = subsystem;
        gyro = ()->0;
        xv = xSupplier;
        yv = ySupplier;
        tv = twistSupplier;
    }
    /** Make mecanum drive command
     *
     * @param subsystem The {@link MecanumDrivebaseSubsystem} for this command
     * @param stick1 The left {@link Stick} for driving
     * @param stick2 The right {@link Stick} for driving
     */
    public MecanumDriveCommand(MecanumDrivebaseSubsystem subsystem, Stick stick1, Stick stick2){
        this(subsystem, stick1.getXSupplier(), stick1.getYSupplier(), stick2.getXSupplier());
    }

    /** Set the drive to be field centric
     *
     * @param gyroSupplier The supplier for the gyro
     * @return this
     */
    public MecanumDriveCommand setFieldCentric(DoubleSupplier gyroSupplier){
        gyro = gyroSupplier;
        return this;
    }
    /** Set the drive to be field centric
     *
     * @param imu The gyro
     * @return this
     */
    public MecanumDriveCommand setFieldCentric(IMU imu){
        return setFieldCentric(imu::gyroHeading);
    }
    /** Set the drive to be field centric
     * @param offset The offset for the gyro
     * @param imu The gyro
     * @return this
     */
    public MecanumDriveCommand setFieldCentric(IMU imu, double offset){
        return setFieldCentric(() -> imu.gyroHeading()+offset);
    }

    /** Drive the robot with {@link MecanumDrivebaseSubsystem}'s joystick drive with or without gyro function
     *
     */
    @Override
    public void execute() {
        subsystem.joystickDriveWithGyro(xv.getAsDouble(), yv.getAsDouble(), tv.getAsDouble(), gyro.getAsDouble());
    }
}

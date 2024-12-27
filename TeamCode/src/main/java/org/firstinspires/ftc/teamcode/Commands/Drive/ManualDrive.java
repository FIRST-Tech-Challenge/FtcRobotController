package org.firstinspires.ftc.teamcode.Commands.Drive;


import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utility.Utils;

// command controls mecanum in manual mode
public class ManualDrive extends CommandBase {

    // PID controller used to counteract rotational drift due to misalignment of wheels
    private PIDController m_headingPID = new PIDController(0.07, 0.0005, 0); // was p0.05
    private Double m_PIDTarget = null;    // Use Double class so it can be set to null
    private long m_pidDelay = -1;

    double powerFactor;
    double basePowerFacter = 0.45;
    double boostPowerFacter = 0.55;

    final double MAX_ACCEL = 1.0;  // max accel in m/s2

    double old_dX, old_dY;

    ElapsedTime deltat;

    // constructor
    public ManualDrive() {

        // this command requires mecanum drive subsystem
        addRequirements(RobotContainer.drivesystem);

        deltat = new ElapsedTime();
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        m_PIDTarget = null;
        m_pidDelay = 10;
        old_dX=0.0;
        old_dY = 0.0;
        deltat.reset();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

        // get joystick input - for competition
        double dX = RobotContainer.ActiveOpMode.gamepad1.left_stick_x;
        double dY = -RobotContainer.ActiveOpMode.gamepad1.left_stick_y;
        double omega = -3.0 * RobotContainer.ActiveOpMode.gamepad1.right_stick_x;
        double speedTrigger = RobotContainer.ActiveOpMode.gamepad1.right_trigger;

        if (RobotContainer.isRedAlliance==false){
            dX = dX * -1;
            dY = dY * -1;
        }

        // implement dead-zoning of joystick inputs
        dX = Math.abs(dX) > 0.05 ? dX : 0;
        dY = Math.abs(dY) > 0.05 ? dY : 0;
        omega = Math.abs(omega) > 0.05 ? omega : 0;

        // --------- Correct robot angle for gyro angle wander --------
        if(omega == 0.0 && !RobotContainer.ActiveOpMode.gamepad1.back)
        {
            if (m_pidDelay > 0)
                m_pidDelay --;
            else
            {
                // If the target is unset, set it to current heading
                if(m_PIDTarget == null)
                {
                    m_PIDTarget = RobotContainer.gyro.getYawAngle();
                    m_headingPID.reset(); // Clear existing integral term as may accumulate while not in use
                }

                omega = m_headingPID.calculate(Utils.AngleDifference(m_PIDTarget, RobotContainer.gyro.getYawAngle()));
            }
        }
        else
        {
            // there is rotational input, or gyro reset was pressed, set target to null so it's properly reset next time
            m_PIDTarget = null;
            m_pidDelay = 10;
        }
        // --------- End Correct robot angle for gyro angle wander --------


        powerFactor = basePowerFacter + (speedTrigger * boostPowerFacter);
        // Since the drive was shifted to closed loop (i.e. requested velocities), change joystick input max values
        // to MAX_SPEED values.
        powerFactor = powerFactor * RobotContainer.drivesystem.MAX_SPEED;

        // include power factor to get full x,y and omega speeds beings requested
        dX *= powerFactor;
        dY *= powerFactor;
        omega *= powerFactor;

        // limit acceleration of robot to MAX_ACCEL - to reduce change of wheel-slide
        double t = deltat.seconds();
        if ((dX > old_dX + t*MAX_ACCEL) && (old_dX>0))
            dX = old_dX + t*MAX_ACCEL;
        if ((dX <  old_dX - t*MAX_ACCEL) && (old_dX<0))
            dX = old_dX - t*MAX_ACCEL;

        if ((dY > old_dY + t*MAX_ACCEL) && (old_dY>0))
            dY = old_dY + t*MAX_ACCEL;
        if ((dY <  old_dY - t*MAX_ACCEL) && (old_dY<0))
            dY = old_dY - t*MAX_ACCEL;

        // save speeds for use next time
        old_dX = dX;
        old_dY = dY;
        deltat.reset();

        // drive robot
        RobotContainer.drivesystem.FieldDrive(dX, dY, omega, 1.0);
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return false;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}
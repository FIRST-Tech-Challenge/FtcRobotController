package org.firstinspires.ftc.teamcode.Commands;


import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utility.Utils;

// command controls mecanum in manual mode
public class ManualDrive extends CommandBase {

    // PID controller used to counteract rotational drift due to misalignment of wheels
    private PIDController m_headingPID = new PIDController(0.05, 0.0005, 0);
    private Double m_PIDTarget = null;    // Use Double class so it can be set to null
    private long m_pidDelay = -1;

    double powerFactor;
    double basePowerFacter = 0.65;
    double boostPowerFacter = 0.35;


    // construtcor
    public ManualDrive() {

        // this command requires mecanum drive subsystem
        addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        m_PIDTarget = null;
        m_pidDelay = 10;
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

        // get joystick input - for competition
        double dX = -RobotContainer.ActiveOpMode.gamepad1.left_stick_y;
        double dY = -RobotContainer.ActiveOpMode.gamepad1.left_stick_x;
        double omega = -3.0 * RobotContainer.ActiveOpMode.gamepad1.right_stick_x;
        double speedTrigger = RobotContainer.ActiveOpMode.gamepad1.right_trigger;

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

        // drive robot
        RobotContainer.drivesystem.FieldDrive(dX, dY, omega, powerFactor);
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
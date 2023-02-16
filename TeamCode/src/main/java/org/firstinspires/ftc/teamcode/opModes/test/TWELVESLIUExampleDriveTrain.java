
package org.firstinspires.ftc.teamcode.opModes.test;

        import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
        import com.arcrobotics.ftclib.gamepad.GamepadEx;
        import com.arcrobotics.ftclib.hardware.motors.Motor;
        import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
        import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
        import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;


@TeleOp(name="Drivetrain Demo [12sliu]", group="Demo")
@Disabled
public class TWELVESLIUExampleDriveTrain extends TeleOpModeBase {
    DifferentialDrive m_drive;
    MotorGroup myMotors1;
    MotorGroup myMotors2;

    @Override
    public void setup() {

        //groups motors together for drive
        this.myMotors1 = new MotorGroup(HardwareMapContainer.motor0, HardwareMapContainer.motor1);
        this.myMotors2 = new MotorGroup(HardwareMapContainer.motor2, HardwareMapContainer.motor3);
        this.m_drive = new DifferentialDrive(myMotors1, myMotors2);

        //some of this is unneeded, can remove at a later time
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    public void every_tick() {
        // Code to run in a loop after `PLAY` is pressed
        telemetry.addData("Status", "Running");
        telemetry.addData("X", Inputs.gamepad1.getLeftX());
        telemetry.addData("Y", Inputs.gamepad1.getLeftY());
        telemetry.update();
        //p sure the test motor thing is useless so I deleted it
        m_drive.arcadeDrive(Inputs.gamepad1.getLeftY(), Inputs.gamepad1.getLeftX());
    }
}


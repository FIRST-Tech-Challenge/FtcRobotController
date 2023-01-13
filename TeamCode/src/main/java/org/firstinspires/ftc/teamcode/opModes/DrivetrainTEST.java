
package org.firstinspires.ftc.teamcode.opModes;

        import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
        import com.arcrobotics.ftclib.gamepad.GamepadEx;
        import com.arcrobotics.ftclib.hardware.motors.Motor;
        import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;


@TeleOp(name="SomeName", group="Test")

public class DrivetrainTEST extends TeleOpModeBase {
    //new code, shoved this here, hope it works?
    GamepadEx gamepadEx = new GamepadEx(gamepad1);
    Motor m_motor_1;
    Motor m_motor_2;
    Motor m_motor_3;
    Motor m_motor_4;

    //groups motors together for drive
    MotorGroup myMotors1 = new MotorGroup(m_motor_1, m_motor_2);
    MotorGroup myMotors2 = new MotorGroup(m_motor_3, m_motor_4);
    DifferentialDrive m_drive = new DifferentialDrive(myMotors1, myMotors2);

    @Override
    public void setup() {
        new Motor(hardwareMap, "motorFour");
        new Motor(hardwareMap, "motorThree");
        new Motor(hardwareMap, "motorTwo");
        new Motor(hardwareMap, "motorOne");
        //some of this is unneeded, can remove at a later time
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    public void every_tick() {
        // Code to run in a loop after `PLAY` is pressed
        telemetry.addData("Status", "Running");
        telemetry.update();
        //p sure the test motor thing is useless so I deleted it
        m_drive.arcadeDrive(gamepadEx.getLeftY(),gamepadEx.getLeftX());
    }
}

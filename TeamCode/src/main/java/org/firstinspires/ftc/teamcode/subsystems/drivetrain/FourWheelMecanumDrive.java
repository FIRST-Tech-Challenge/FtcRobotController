package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;

/**
 * for wheel macanum drive train that hold references to 4 wheels and init them.
 */
public class FourWheelMecanumDrive extends BasicDriveTrain {

    protected Motor fL, fR, bL, bR;


    public FourWheelMecanumDrive(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback, boolean revertMotors) {
        super(hardwareMap, gamepad, telemetry, feedback, revertMotors);
    }

    @Override
    protected void createAndInitHardwares(HardwareMap hardwareMap) {
        this.fL = new Motor(hardwareMap, "FL");
        this.bL = new Motor(hardwareMap, "BL");
        this.fR = new Motor(hardwareMap, "FR");
        this.bR = new Motor(hardwareMap, "BR");

        if (revertMotor) {
            this.fR.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            this.bR.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        }

        fL.encoder.reset();
        fR.encoder.reset();
        bL.encoder.reset();
        bR.encoder.reset();

        fL.setRunMode(Motor.RunMode.RawPower);
        fR.setRunMode(Motor.RunMode.RawPower);
        bL.setRunMode(Motor.RunMode.RawPower);
        bR.setRunMode(Motor.RunMode.RawPower);

//        fL.setInverted(false);
//        fR.setInverted(true);
//        bL.setInverted(false);
//        bR.setInverted(true);

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    protected void createDrive() {
        this.drive = new MecanumDrive(this.fL, this.fR, this.bL, this.bR);
    }

    public Motor getfL() {
        return fL;
    }

    public Motor getfR() {
        return fR;
    }

    public Motor getbL() {
        return bL;
    }

    public Motor getbR() {
        return bR;
    }
}

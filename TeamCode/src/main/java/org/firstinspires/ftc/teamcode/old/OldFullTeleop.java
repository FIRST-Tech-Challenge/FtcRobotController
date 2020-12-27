package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drivebase.GyroSensor;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.ringtransfer.BoxFlickerControls;
import org.firstinspires.ftc.teamcode.wobblegoal.ArmControls;
import org.firstinspires.ftc.teamcode.wobblegoal.GripperControls;

@TeleOp(name = "OLD Full Robot Teleop", group = "Linear Opmode")
@Disabled
public class OldFullTeleop extends LinearOpMode {
    public DcMotor intake = null;
    private double _powerIntake = 0.0;
    private boolean _intake = false;
    private boolean _outtake = false;

    private MecanumDrivebase mecanumDrivebase = new MecanumDrivebase();
    private GyroSensor gyroSensor = new GyroSensor();
    //private IntakeControls intakeControls = new IntakeControls();
    private ArmControls armControls = new ArmControls();
    private EasySnowplowControls plowControls = new EasySnowplowControls();
    private BoxFlickerControls flickerControls = new BoxFlickerControls();
    private GripperControls gripperControls = new GripperControls();

    @Override
    public void runOpMode() {

        mecanumDrivebase.initialize(this);
        gyroSensor.initialize(this);
        //intakeControls.initialize(this, telemetry);
        armControls.initialize(this);
        plowControls.initialize(this);
        flickerControls.initialize(this);
        gripperControls.initialize(this);

        intake = hardwareMap.get(DcMotor.class, "Intake");
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0.0);

        // Wait for the start button
        waitForStart();

        mecanumDrivebase.startControl();
        gyroSensor.startControl();
        //intakeControls.startControl();
        armControls.startControl();
        plowControls.startControl();
        flickerControls.startControl();
        gripperControls.startControl();

        while(opModeIsActive()) {

            _outtake = gamepad1.left_bumper;
            _intake = gamepad1.right_bumper;
            if (_outtake == true) {
                _powerIntake = -1.0;
            } else if (_intake == true) {
                _powerIntake = 1.0;
            } else {
                _powerIntake = 0.0;
            }
            intake.setPower(_powerIntake);



            mecanumDrivebase.readController(gamepad1);
            //intakeControls.readController(gamepad1);
            //armControls.readController(gamepad2);
            //plowControls.readController(gamepad2);
            //flickerControls.readController(gamepad2);
            //gripperControls.readController(gamepad2);

            gyroSensor.updateAngles(this);

            mecanumDrivebase.setGyroAngle(gyroSensor.getDirection());

            mecanumDrivebase.whileOpModeIsActive(this);
            //intakeControls.whileOpModeIsActive(this, telemetry);
            //armControls.whileOpModeIsActive(this);
            //plowControls.whileOpModeIsActive(this);
            //flickerControls.whileOpModeIsActive(this);
            //gripperControls.whileOpModeIsActive(this);

            mecanumDrivebase.addTelemetry(telemetry);
            //armControls.addTelemetry(telemetry);
            //intakeControls.addTelemetry(telemetry);
            telemetry.update();
            idle();
        }

        mecanumDrivebase.stop();
        //intakeControls.stop();
        armControls.stop();
        intake.setPower(0.0);

    }

}
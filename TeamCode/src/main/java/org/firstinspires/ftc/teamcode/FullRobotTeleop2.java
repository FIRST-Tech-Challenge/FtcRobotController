package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivebase.GyroSensor;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.intake.ComplicatedSnowplowControls;
import org.firstinspires.ftc.teamcode.intake.IntakeControls;
import org.firstinspires.ftc.teamcode.odometry.OdometryControls;
import org.firstinspires.ftc.teamcode.ringtransfer.BoxFlickerControls;
import org.firstinspires.ftc.teamcode.wobblegoal.ArmControls;
import org.firstinspires.ftc.teamcode.wobblegoal.GripperControls;

@TeleOp(name = "Full Robot Teleop", group = "Linear Opmode")
//@Disabled
public class FullRobotTeleop2 extends LinearOpMode {

    double _time = 0.0;

    private MecanumDrivebase mecanumDrivebase = new MecanumDrivebase();
    private GyroSensor gyroSensor = new GyroSensor();
    private ArmControls armControls = new ArmControls();
    private IntakeControls intakeControls = new IntakeControls();
    private ComplicatedSnowplowControls plowControls = new ComplicatedSnowplowControls();
    private GripperControls gripperControls = new GripperControls();
    private BoxFlickerControls flickerControls = new BoxFlickerControls();
    private OdometryControls odometryControls = new OdometryControls();
    //private TimerControls timer = new TimerControls();

    @Override
    public void runOpMode() {

        mecanumDrivebase.initialize(this);
        gyroSensor.initialize(this);
        plowControls.initialize(this);
        armControls.initialize(this);
        intakeControls.initialize(this);
        gripperControls.initialize(this);
        flickerControls.initialize(this);
        odometryControls.initialize(this);
        //timer.initialize(this);

        // Wait for the start button
        waitForStart();

        mecanumDrivebase.startControl();
        gyroSensor.startControl();
        gripperControls.startControl();
        flickerControls.startControl();
        armControls.startControl();
        intakeControls.startControl();
        odometryControls.startControl();
        //timer.startControl();
        //plowControls.startControl();

        while(opModeIsActive()) {

            _time = getRuntime();

            mecanumDrivebase.readController(gamepad1);
            intakeControls.readController(gamepad1);
            armControls.readController(gamepad2);
            plowControls.readController(gamepad2);
            flickerControls.readController(gamepad2);
            gripperControls.readController(gamepad2);

            gyroSensor.updateAngles(this);

            mecanumDrivebase.setGyroAngle(gyroSensor.getDirection());

            mecanumDrivebase.whileOpModeIsActive(this);
            intakeControls.whileOpModeIsActive(this);
            armControls.whileOpModeIsActive(this);
            plowControls.whileOpModeIsActive(this, _time);
            //plowControls.whileOpModeIsActive(this);
            flickerControls.whileOpModeIsActive(this);
            gripperControls.whileOpModeIsActive(this);
            odometryControls.whileOpModeIsActive(this);

            //mecanumDrivebase.addTelemetry(telemetry);
            armControls.addTelemetry(telemetry);
            //intakeControls.addTelemetry(telemetry);
            plowControls.addTelemetry(telemetry);
            odometryControls.addTelemetry(telemetry);
            //timer.addTelemetry(telemetry);
            telemetry.addData("Opmode Timer (ms)", _time);
            telemetry.update();
            idle();
        }

        mecanumDrivebase.stop();
        intakeControls.stop();
        armControls.stop();
        odometryControls.stop();

    }

}
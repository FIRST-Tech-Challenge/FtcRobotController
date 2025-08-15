package org.firstinspires.ftc.teamcode.IntoTheDeep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp
public class FODXDrive extends OpMode {
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorL;
    DcMotor motorR;
    Servo garra;
    Servo intake;
    IMU imu;

    @Override
    public void init() {

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));
        imu.initialize(parameters);

        motorFL = hardwareMap.get(DcMotor.class, "FL");
        motorFR = hardwareMap.get(DcMotor.class, "FR");
        motorBL = hardwareMap.get(DcMotor.class, "BL");
        motorBR = hardwareMap.get(DcMotor.class, "BR");
        motorL = hardwareMap.get(DcMotor.class, "AL");
        motorR = hardwareMap.get(DcMotor.class, "AR");
        garra = hardwareMap.get(Servo.class, "garra");
        intake = hardwareMap.get(Servo.class, "intake");

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorL.setTargetPosition(0);
        motorR.setTargetPosition(0);
        garra.setPosition(1);

        motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorL.setPower(0.2);
        motorR.setPower(0.2);

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        telemetry.addData("Hardware: ", "Initialized");
    }

    /*public void init_loop() {}
    public void start(){}*/

    @Override
    public void loop(){
        telemetry.addData("Hardware: ", "Running");

        if (gamepad2.dpad_up){
            motorL.setTargetPosition(100);
            motorR.setTargetPosition(100);
        } else if (gamepad2.dpad_down) {
            motorL.setTargetPosition(0);
            motorR.setTargetPosition(0);
        }

        if (gamepad2.x) {
            garra.setPosition(0);
        } else if (gamepad2.y) {
            garra.setPosition(1);
        }

        if (gamepad2.a) {
            intake.setPosition(0);
        } else if (gamepad2.b) {
            intake.setPosition(0.35);
        }

        double drive = gamepad1.left_stick_y;  // frente e atrás
        double turn = -gamepad1.right_stick_x;  // gira
        double strafe = -gamepad1.left_stick_x; // direita e esquerda

        double max = Math.max(Math.abs(strafe) + Math.abs(drive) + Math.abs(turn), 1);

        double drivePower = -(0.5 * gamepad1.right_trigger) + 1;
        if(gamepad1.left_bumper) imu.resetYaw();

        telemetry.addLine("Angulo do robô: "+ imu.getRobotYawPitchRollAngles().getYaw());

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double adjustedStrafe = drive * Math.sin(heading) + strafe * Math.cos(heading);
        double adjustedDrive = drive * Math.cos(heading) - strafe * Math.sin(heading);

        motorFL.setPower(((adjustedDrive + adjustedStrafe + turn) / max) * drivePower);
        motorFR.setPower(((adjustedDrive + adjustedStrafe - turn) / max) * drivePower);
        motorBL.setPower(((adjustedDrive - adjustedStrafe + turn) / max) * drivePower);
        motorBR.setPower(((adjustedDrive - adjustedStrafe - turn) / max) * drivePower);
        telemetry.update();
    }
}

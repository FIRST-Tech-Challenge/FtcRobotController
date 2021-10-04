package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;  //This is the package for controlling the IMU
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.api.movement.old.ChassisMovementCode;

//Created by mostly Patrick, partly Ethan
@Autonomous
public class firstAutonomousMode extends LinearOpMode {

    private Blinker Control_Hub;
    private Blinker expansion_Hub_2;

    //motors

    private BNO055IMU imu;

    private DigitalChannel switch_;

    ElapsedTime servoTimer = new ElapsedTime();

    enum OperState {
        FIRSTMOVE,
        SECONDMOVESETUP,
        SECONDMOVE,
        THIRDMOVESETUP,
        THIRDMOVE,
        STARTLAUNCHER,
        FOURTHMOVESETUP,
        FOURTHMOVE,
        SHOOT1,
        Pressed,
        firsttimer,
        Load,
        secondtimer,
        ResetPosition,
        FIFTHMOVESETUP,
        FIFTHMOVE,
        SIXTHMOVESETUP,
        SIXTHMOVE
    }

    @Override
    public void runOpMode() {
        LauncherCode.Launcher launcher = new LauncherCode.Launcher();
        LifterCode.Lifter lift = new LifterCode.Lifter();
        ChassisMovementCode.Chassis chassis = new ChassisMovementCode.Chassis();
        firstAutonomousMode.OperState driveOpState = firstAutonomousMode.OperState.FIRSTMOVE;


        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        lift.LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");

        launcher.LaunchMotor = hardwareMap.get(DcMotor.class, "LaunchMotor");
        launcher.LaunchServo = hardwareMap.get(Servo.class, "LaunchServo");
        chassis.imu = hardwareMap.get(BNO055IMU.class, "imu");
        chassis.front_left_wheel = hardwareMap.get(DcMotor.class, "front left wheel");
        chassis.front_right_wheel = hardwareMap.get(DcMotor.class, "front right wheel");
        chassis.back_left_wheel = hardwareMap.get(DcMotor.class, "back left wheel");
        chassis.back_right_wheel = hardwareMap.get(DcMotor.class, "back right wheel");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();  //in wrong spot--where is better?
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        chassis.imu.initialize(parameters);
        double drivePreset = 0;
        double rotationGoal = chassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        double originalRotation = rotationGoal;
        double shootWait = 0;
        chassis.SetRotation(chassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
        waitForStart();
        servoTimer.reset();

        while (opModeIsActive()) {
            chassis.SetRotation(chassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
            launcher.LauncherRun(1);
/*
            switch(driveOpState) {
                case FIRSTMOVE:
                    telemetry.addLine("FIRSTMOVE");


                    if (servoTimer.time()  >= 5) {

                        driveOpState = firstAutonomousMode.OperState.SECONDMOVESETUP;
                    }

                    break;

                    
                case SECONDMOVESETUP:
                    chassis.Encoders();
                    chassis.ZeroEncoders();
                    chassis.Encoders();
                    chassis.SetAxisMovement();
                    drivePreset = chassis.trueDrive - 60;
                    rotationGoal = chassis.zAngle;
                    driveOpState = firstAutonomousMode.OperState.SECONDMOVE;

                    break;

                case SECONDMOVE:
                    telemetry.addLine("SECONDMOVE");
                    telemetry.addData("Drive Preset: ", drivePreset);
                    telemetry.addData("drive", chassis.trueDrive);
                    chassis.Encoders();
                    chassis.SetAxisMovement();
                    chassis.ForwardAndBackward(drivePreset);
                    rotationGoal += -0.02;




                    if ((Math.abs(chassis.zAngle - rotationGoal) >= 2)) {
                        chassis.SetMotors(0, 0, chassis.CorrectRotation(chassis.zAngle, rotationGoal));
                        chassis.Drive();
                    }

                    if (Math.abs(drivePreset - chassis.trueDrive) <= 0.2) {
                        chassis.front_left_wheel.setPower(-0.01);
                        chassis.front_right_wheel.setPower(-0.01);
                        chassis.back_right_wheel.setPower(-0.01);
                        chassis.back_left_wheel.setPower(-0.01);
                        driveOpState = firstAutonomousMode.OperState.THIRDMOVESETUP;
                    }

                    break;


                case THIRDMOVESETUP:
                    telemetry.addLine("THIRDMOVESETUP");
                    if ((Math.abs(chassis.zAngle - rotationGoal) >= 2)) {
                        chassis.SetMotors(0, 0, chassis.CorrectRotation(chassis.zAngle, rotationGoal));
                        chassis.Drive();
                        chassis.Encoders();
                        chassis.ZeroEncoders();
                        chassis.SetAxisMovement();
                        rotationGoal = chassis.zAngle;
                    }
                    else {
                        chassis.front_left_wheel.setPower(-0.01);
                        chassis.front_right_wheel.setPower(-0.01);
                        chassis.back_right_wheel.setPower(-0.01);
                        chassis.back_left_wheel.setPower(-0.01);

                        servoTimer.reset();

                        driveOpState = firstAutonomousMode.OperState.THIRDMOVE;
                    }
                    break;


                case THIRDMOVE:
                    telemetry.addLine("THIRDMOVE");
                    ;

                    if (servoTimer.time() >= 2) {

                        driveOpState = firstAutonomousMode.OperState.STARTLAUNCHER;
                    }

                    break;

                case STARTLAUNCHER:
                    launcher.LauncherToggle();
                    driveOpState = firstAutonomousMode.OperState.FOURTHMOVESETUP;


                case FOURTHMOVESETUP:
                    telemetry.addLine("FIFTHMOVESETUP");
                    telemetry.addData("chassis.zAngle", chassis.zAngle);
                    telemetry.addData("originalRotaion",originalRotation);
                    telemetry.addData("goal for rotation", originalRotation-180);
                    if ((Math.abs(chassis.zAngle - (originalRotation-182)) >= 2)) {
                        chassis.SetMotors(0, 0, chassis.CorrectRotation(chassis.zAngle, (originalRotation-182)));
                        chassis.Drive();
                        chassis.Encoders();
                        chassis.ZeroEncoders();
                        chassis.Encoders();
                        chassis.SetAxisMovement();
                        rotationGoal = chassis.zAngle;
                    }
                    else {
                        chassis.front_left_wheel.setPower(-0.01);
                        chassis.front_right_wheel.setPower(-0.01);
                        chassis.back_right_wheel.setPower(-0.01);
                        chassis.back_left_wheel.setPower(-0.01);

                        drivePreset = chassis.trueStrafe - 50;


                        driveOpState = firstAutonomousMode.OperState.FOURTHMOVE;
                    }

                    break;

                case FOURTHMOVE:
                    telemetry.addLine("FOURTHMOVE");
                    telemetry.addData("Drive Preset: ", drivePreset);
                    telemetry.addData("strafe", chassis.trueStrafe);
                    chassis.Encoders();
                    chassis.SetAxisMovement();
                    chassis.LeftAndRight(drivePreset);





                    if ((Math.abs(chassis.zAngle - rotationGoal) >= 2)) {
                        chassis.SetMotors(0, 0, chassis.CorrectRotation(chassis.zAngle, rotationGoal));
                        chassis.Drive();
                    }

                    if (Math.abs(drivePreset - chassis.trueStrafe) <= 11) {
                        chassis.front_left_wheel.setPower(-0.01);
                        chassis.front_right_wheel.setPower(-0.01);
                        chassis.back_right_wheel.setPower(-0.01);
                        chassis.back_left_wheel.setPower(-0.01);
                        servoTimer.reset();
                        driveOpState = firstAutonomousMode.OperState.SHOOT1;
                    }

                    break;

                case SHOOT1:
                    if (shootWait < 6) {
                        if (servoTimer.time() > 0.5) {
                            driveOpState = firstAutonomousMode.OperState.firsttimer;
                        }
                    }
                    else {
                        driveOpState = firstAutonomousMode.OperState.FIFTHMOVESETUP;
                    }



                    break;
                case firsttimer:
                    servoTimer.reset();
                    driveOpState = firstAutonomousMode.OperState.Load;
                    break;

                case Load:
                    launcher.Shoot();
                    if (servoTimer.time() >= 0.15) {
                        driveOpState = firstAutonomousMode.OperState.secondtimer;
                    }
                    break;

                case secondtimer:
                    servoTimer.reset();
                    driveOpState = firstAutonomousMode.OperState.ResetPosition;
                    break;

                case ResetPosition:
                    launcher.Reload();
                    if (servoTimer.time() >= 0.15) {
                        shootWait += 1;
                        servoTimer.reset();
                        driveOpState = firstAutonomousMode.OperState.SHOOT1;
                    }
                    break;

                case FIFTHMOVESETUP:
                    telemetry.addLine("FIFTHMOVESETUP");
                    if ((Math.abs(chassis.zAngle - originalRotation) >= 2)) {
                        chassis.SetMotors(0, 0, chassis.CorrectRotation(chassis.zAngle, originalRotation));
                        chassis.Drive();
                        chassis.Encoders();
                        chassis.ZeroEncoders();
                        chassis.SetAxisMovement();
                        rotationGoal = chassis.zAngle;
                    }
                    else {
                        chassis.front_left_wheel.setPower(-0.01);
                        chassis.front_right_wheel.setPower(-0.01);
                        chassis.back_right_wheel.setPower(-0.01);
                        chassis.back_left_wheel.setPower(-0.01);

                        drivePreset = chassis.trueDrive + 10;

                        driveOpState = firstAutonomousMode.OperState.FIFTHMOVE;
                    }
                    break;
            }
 */



            telemetry.update();
        }
    }
}

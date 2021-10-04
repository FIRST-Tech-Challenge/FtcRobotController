package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
public class rotate360onA extends LinearOpMode {

    private Blinker Control_Hub;
    private Blinker expansion_Hub_2;

    //motors

    private BNO055IMU imu;

    private DigitalChannel switch_;

    ElapsedTime servoTimer = new ElapsedTime();

    enum OperState {
        SETUP,
        rotate,
        DRIVEANDROTATE
    }

    @Override
    public void runOpMode() {
        LauncherCode.Launcher launcher = new LauncherCode.Launcher();
        LifterCode.Lifter lift = new LifterCode.Lifter();
        ChassisMovementCode.Chassis chassis = new ChassisMovementCode.Chassis();
        rotate360onA.OperState driveOpState = rotate360onA.OperState.SETUP;


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
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        chassis.imu.initialize(parameters);
        double drivePreset = 0;
        double rotationGoal = chassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double originalRotation = rotationGoal;
        double shootWait = 0;
        double initialLeft = 0;
        double initialBack = 0;
        double initialRight = 0;
        double directionSwitch = -1;

        double AveragedArray;
        double totalLeft = 0;
        double totalBack = 0;
        double totalRight =0;
        ElapsedTime driveTimer = new ElapsedTime();
        int index = 0;
        int ArraySize = 500; //Allows for easy way to change the size of the array that affects all of the code
        double[] movementArrayLeft = new double [ArraySize];
        double[] movementArrayBack = new double [ArraySize];
        double[] movementArrayRight = new double [ArraySize];


        chassis.SetRotation(chassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        waitForStart();
        servoTimer.reset();

        while (opModeIsActive()) {
            chassis.SetRotation(chassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            launcher.LauncherRun(1);

            switch (driveOpState) {
                case SETUP:
                    telemetry.addData("Right Encoder", chassis.back_right_wheel.getCurrentPosition());
                    telemetry.addData("Left Encoder", -chassis.front_right_wheel.getCurrentPosition());
                    telemetry.addData("Back Encoder", chassis.front_left_wheel.getCurrentPosition());
                    telemetry.addData("Initial Right Encoder", initialRight);
                    telemetry.addData("Initial Left Encoder", initialLeft);
                    telemetry.addData("Initial Back Encoder", initialBack);
                    telemetry.addData("Right Encoder Difference", chassis.back_right_wheel.getCurrentPosition()-initialRight);
                    telemetry.addData("Left Encoder Difference", -chassis.front_right_wheel.getCurrentPosition()-initialLeft);
                    telemetry.addData("Back Encoder Difference", chassis.front_left_wheel.getCurrentPosition()-initialBack);
                    telemetry.addData("Right Encoder Average Difference", totalRight / (index-1));
                    telemetry.addData("Left Encoder Average Difference", totalLeft / (index-1));
                    telemetry.addData("Back Encoder Average Difference", totalBack / (index-1));
                    telemetry.addData("Right Encoder Multiplier", (totalBack / (index-1)) / (totalRight / (index-1)) );
                    telemetry.addData("Left Encoder Multiplier", (totalBack / (index-1)) / (totalLeft / (index-1)) );
                    telemetry.addData("Back Encoder Multiplier" , 1);

                    if (gamepad1.a) {
                        if (index >= (ArraySize - 1)) {
                            index = 0;
                        }
                        else {
                            index++;
                        }
                        movementArrayLeft[index] = -chassis.front_right_wheel.getCurrentPosition()-initialLeft;
                        movementArrayBack[index] = chassis.front_left_wheel.getCurrentPosition()-initialBack;
                        movementArrayRight[index] = chassis.back_right_wheel.getCurrentPosition()-initialRight;

                        initialLeft = -chassis.front_right_wheel.getCurrentPosition();
                        initialBack = chassis.front_left_wheel.getCurrentPosition();
                        initialRight = chassis.back_right_wheel.getCurrentPosition();
                        rotationGoal = chassis.zAngle;
                        driveOpState = rotate360onA.OperState.rotate;
                    }
                    if (gamepad1.right_trigger != 0) {
                        initialLeft = -chassis.front_right_wheel.getCurrentPosition();
                        initialBack = chassis.front_left_wheel.getCurrentPosition();
                        initialRight = chassis.back_right_wheel.getCurrentPosition();
                        rotationGoal = chassis.zAngle;
                        driveTimer.reset();
                        directionSwitch = directionSwitch * -1;
                        driveOpState = rotate360onA.OperState.DRIVEANDROTATE;
                    }
                    break;

                case DRIVEANDROTATE:
                    if (driveTimer.time() <= 8) {
                        telemetry.addLine("Drive and Rotate");
                        chassis.SetAxisMovement();
                        //double drive = -this.gamepad1.left_stick_y;
                        //strafe = this.gamepad1.left_stick_x;
                        double rotate = -this.gamepad1.right_stick_x;

                        chassis.SetPresetAxis();

                        chassis.SetMotors(0.5 * directionSwitch, 0, 0);
                        chassis.Drive();
                        chassis.SetTrueAxis();
                    }
                    else {
                        chassis.front_left_wheel.setPower(-0.01);
                        chassis.front_right_wheel.setPower(-0.01);
                        chassis.back_right_wheel.setPower(-0.01);
                        chassis.back_left_wheel.setPower(-0.01);
                        if (index >= (ArraySize - 1)) {
                            index = 0;
                        }
                        else {
                            index++;
                        }
                        //movementArrayLeft[index] = Math.abs((-chassis.front_right_wheel.getCurrentPosition()-initialLeft)*chassis.leftEncoderMultiplier);
                        //movementArrayBack[index] = Math.abs((chassis.front_left_wheel.getCurrentPosition()-initialBack)*chassis.backEncoderMultiplier);
                        //movementArrayRight[index] = Math.abs((chassis.back_right_wheel.getCurrentPosition()-initialRight)*chassis.rightEncoderMultiplier);
                        totalLeft = 0;
                        totalBack = 0;
                        totalRight = 0;
                        for (int i = 0; i <= (ArraySize - 1); i++) {
                            totalLeft = totalLeft + movementArrayLeft[i];
                            totalRight = totalRight + movementArrayRight[i];
                            totalBack = totalBack + movementArrayBack[i];
                        }
                    }
                    break;

                case rotate:
                    telemetry.addLine("Rotate");
                    if ((Math.abs(chassis.zAngle - (rotationGoal - 180)) >= 2)) {
                        chassis.SetMotors(0, 0, chassis.CorrectRotation(chassis.zAngle, (rotationGoal - 180),1));
                        chassis.Drive();
                        chassis.SetAxisMovement();
                        chassis.ZeroEncoders();
                        chassis.SetAxisMovement();

                    } else {
                        chassis.front_left_wheel.setPower(-0.01);
                        chassis.front_right_wheel.setPower(-0.01);
                        chassis.back_right_wheel.setPower(-0.01);
                        chassis.back_left_wheel.setPower(-0.01);
                        if (index >= (ArraySize - 1)) {
                            index = 0;
                        }
                        else {
                            index++;
                        }
                        //movementArrayLeft[index] = (-chassis.front_right_wheel.getCurrentPosition()-initialLeft)*chassis.leftEncoderMultiplier;
                        //movementArrayBack[index] = (chassis.front_left_wheel.getCurrentPosition()-initialBack)*chassis.backEncoderMultiplier;
                        //movementArrayRight[index] = (chassis.back_right_wheel.getCurrentPosition()-initialRight)*chassis.rightEncoderMultiplier;
                        totalLeft = 0;
                        totalBack = 0;
                        totalRight = 0;
                        for (int i = 0; i <= (ArraySize - 1); i++) {
                            totalLeft = totalLeft + movementArrayLeft[i];
                            totalRight = totalRight + movementArrayRight[i];
                            totalBack = totalBack + movementArrayBack[i];
                        }
                        driveOpState = rotate360onA.OperState.SETUP;
                    }

                    break;

                    /*
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


            }/*cdg*/

            telemetry.update();
        }
    }
}

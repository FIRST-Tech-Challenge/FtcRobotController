package org.firstinspires.ftc.teamcode;
import android.os.DropBoxManager;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.lang.Math;
@TeleOp
public class ChassisMovementCode { //PATRICK IS RACIST AAA DONT LET HIM SEE THIS!!!!
    public static class Chassis {
        ElapsedTime movementTimer = new ElapsedTime();
        public DcMotor back_right_wheel;
        public DcMotor front_right_wheel;
        public DcMotor back_left_wheel;
        public DcMotor front_left_wheel;
        public BNO055IMU imu;
        private DigitalChannel switch_;

        public double frontLeft;
        public double frontRight;
        public double backLeft;
        public double backRight;
        double rightEncoder;
        double leftEncoder;
        double backEncoder;
        double clearRight = 0;
        double clearLeft = 0;
        double clearBack = 0;
        public double clearRotate = 0;
        double fieldLength = 141;
        double robotLength = 17.25;
        double robotWidth = 17.375;
        double countsPerRotation = 360;
        public double trueDrive;

        public double trueStrafe;
        double slowIntensity = 10;
        public double trueRotate;
        public double backRightMultiplier = 1;
        public double backLeftMultiplier = 1;
        public double frontRightMultiplier = 0.9;
        public double frontLeftMultiplier = 0.9;
        double oldZAngel =0;
        double newZAngle = 0;
        double rotations=0;
        public double zAngle = 0;
        public double presetX = 0;
        public double presetY = 0;
        public double trueX = 0;
        public double trueY = 0;
        public double clearDrive = 0;
        public double clearStrafe = 0;
        public double tau = 6.28318530718;

        double IMUDrive = 0;
        double IMUStrafe = 0;



        double drive;
        double driveSpeedMultiplier = 1;
        int isDrive = 0;

        double strafe;
        double strafeSpeedMultiplier = 1;
        int isStrafe = 0;

        double rotation;
        int isRotate= 0;

        double drivePreset = 0;
        double strafePreset = 0;
        double rotationPreset = 0;

        boolean isDone = false;

        public void SetPresetMovement(double Preset_Drive, double Drive_Speed_Multiplier, double Preset_Strafe, double Strafe_Speed_Multiplier, double Preset_Rotation) {
            drivePreset = trueDrive + Preset_Drive;
            strafePreset = trueStrafe + Preset_Strafe;
            rotationPreset = Preset_Rotation;
            strafeSpeedMultiplier = Strafe_Speed_Multiplier;
            driveSpeedMultiplier = Drive_Speed_Multiplier;

        }

        public double StrafeMovement (double currentStrafe, double strafeGoal) {

            strafe = Math.signum(strafeGoal - currentStrafe) * Math.max(0.15, Math.abs((strafeGoal - currentStrafe) / strafeGoal));
            return (strafe);
        }

        public double DriveMovement (double currentDrive, double driveGoal) {

            drive = Math.signum(driveGoal - currentDrive) * Math.max(0.15, Math.abs((driveGoal - currentDrive) / driveGoal));
            return (drive);
        }

        public double CorrectRotation(double currentRotation, double rotationGoal, double speed) {

            rotation = Math.signum(rotationGoal - currentRotation) * (Math.max(0.2, speed * Math.abs((rotationGoal - currentRotation) / 180)));
            return (rotation);
        }

        public boolean MoveToLocation () {
            this.SetAxisMovement();

            if (drivePreset == 0) {
                drivePreset = 0.0000000000000000000000001;
            }
            if (strafePreset == 0) {
                strafePreset = 0.0000000000000000000000001;
            }

            if (Math.abs(drivePreset - trueDrive) >= 2) {
                drive = Math.signum(drivePreset - trueDrive) * Math.max(0.2, driveSpeedMultiplier * Math.abs((drivePreset - trueDrive) / drivePreset));
            } else {isDrive = 1; drive = 0;}

            if (Math.abs(strafePreset - trueStrafe) >= 1) {
                strafe = Math.signum(strafePreset - trueStrafe) * Math.max(0.2, strafeSpeedMultiplier * Math.abs((strafePreset - trueStrafe) / strafePreset));
            } else {isStrafe = 1; strafe = 0;}

            if ((Math.abs(zAngle - rotationPreset) >= 2)) {
                rotation = Math.signum(rotationPreset - zAngle) * (Math.max(0.2, Math.abs((rotationPreset - zAngle) / 180)));
            } else {isRotate = 1; rotation = 0;}

            this.SetMotors(drive,strafe,rotation);
            this.Drive();

            if ((isDrive == 1) & (isRotate == 1) & (isStrafe == 1)) {
                isDrive = 0;
                isRotate = 0;
                isStrafe = 0;
                front_left_wheel.setPower(-0.01);
                front_right_wheel.setPower(-0.01);
                back_right_wheel.setPower(-0.01);
                back_left_wheel.setPower(-0.01);
                isDone = true;
            }
            else {
                isDrive = 0;
                isRotate = 0;
                isStrafe = 0;
                isDone = false;
            }

            return (isDone);
        }

        public void SetTrueAxis() {
            trueX = trueStrafe*Math.cos((trueRotate))+trueDrive*(Math.cos(trueRotate+(Math.PI/2))) + presetX;
            trueY = trueStrafe*Math.sin((trueRotate))+trueDrive*(Math.sin(trueRotate+(Math.PI/2))) + presetY;
        }

        public void SetPresetAxis() {
            presetX = trueX;
            presetY = trueY;
            clearDrive = (rightEncoder + leftEncoder) / 2;
            clearStrafe = backEncoder - (rightEncoder - leftEncoder) / 2;
        }


        public void SetAxisMovement() {
            rightEncoder = back_right_wheel.getCurrentPosition() / 360 * 1.173150521 - clearRight;
            leftEncoder = -front_right_wheel.getCurrentPosition() / 360 * 1.178221633 - clearLeft;
            backEncoder = front_left_wheel.getCurrentPosition() / 360 * 1.17584979 - clearBack;
            trueDrive = ((rightEncoder + leftEncoder) / 2)-clearDrive;
            trueStrafe = (backEncoder - (rightEncoder - leftEncoder) / 2)-clearStrafe;
            trueRotate = (((rightEncoder - leftEncoder) / 2)*0.12877427457 /* <---- see comment below*/)-clearRotate;
            /*
            the number 0.12877427457 was calculated through taking several measurements with the im
            and with the rotate encoders, then I divided the imu measurement by the encoder measurement
            and got the average of all my attempts giving me a multiplier to go from encoders counts
            to degrees, 7.3305, then I multiplied that by the rotation to radians multiplier, pi/180,
            then I ran the robot and rotated 360 degrees several times to get measurements with the
            new rotate values then I divided tau by the new rotation measurements and got the average
            of all my attempts to get the multiplier to go from my new measurements to true tau,
            1.00651012115, then I multiplied my new multiplier by my old multiplier to get my final
            multiplier, 0.12877427457
             */

            if (trueRotate >= tau) {
                clearRotate += tau;
            }
            if (trueRotate < 0) {
                clearRotate -= tau;
            }

        }

        public void ForwardAndBackward(double drivePreset) {
            front_right_wheel.setPower(frontRightMultiplier * Math.signum(drivePreset - trueDrive) * Math.max(0.15, Math.abs((drivePreset - trueDrive) / drivePreset)));
            front_left_wheel.setPower(frontLeftMultiplier * Math.signum(drivePreset - trueDrive) * Math.max(0.15, Math.abs((drivePreset - trueDrive) / drivePreset)));
            back_left_wheel.setPower(-1 * backLeftMultiplier * Math.signum(drivePreset - trueDrive) * Math.max(0.15, Math.abs((drivePreset - trueDrive) / drivePreset)));
            back_right_wheel.setPower(backRightMultiplier * Math.signum(drivePreset - trueDrive) * Math.max(0.15, Math.abs((drivePreset - trueDrive) / drivePreset)));

        }

        public void SetRotation(double imuZAngle) {
            oldZAngel = newZAngle;
            newZAngle = imuZAngle;
            if ((Math.signum(newZAngle) != Math.signum(oldZAngel)) & Math.abs(newZAngle) > 50) {
                rotations += Math.signum(oldZAngel);
            }
            zAngle = (newZAngle + rotations*360);
        }

        public void LeftAndRight(double drivePreset) {
            front_right_wheel.setPower(-1 * frontRightMultiplier * Math.signum(drivePreset - trueStrafe) * Math.max(0.15, Math.abs((drivePreset - trueStrafe) / drivePreset)));
            front_left_wheel.setPower(frontLeftMultiplier * Math.signum(drivePreset - trueStrafe) * Math.max(0.15, Math.abs((drivePreset - trueStrafe) / drivePreset)));
            back_left_wheel.setPower(backLeftMultiplier * Math.signum(drivePreset - trueStrafe) * Math.max(0.15, Math.abs((drivePreset - trueStrafe) / drivePreset)));
            back_right_wheel.setPower(backRightMultiplier * Math.signum(drivePreset - trueStrafe) * Math.max(0.15, Math.abs((drivePreset - trueStrafe) / drivePreset)));

        }


        public void ZeroEncoders() {
            clearRight = back_right_wheel.getCurrentPosition() / 360 * 1.173150521;
            clearLeft = -front_right_wheel.getCurrentPosition() / 360 * 1.178221633;
            clearBack = front_left_wheel.getCurrentPosition() / 360 * 1.17584979;
        }

        public void Encoders() {
            /*telemetry.addData("True back", backEncoder / 360 * 1.17584979);
            telemetry.addData("True right", rightEncoder / 360 * 1.173150521);
            telemetry.addData("True left", leftEncoder / 360 * 1.178221633);
            telemetry.addData("Right Encoder CM", rightEncoder);
            telemetry.addData("Left Encoder CM", leftEncoder);
            telemetry.addData("Back Encoder CM", backEncoder);
            telemetry.addData("Right Encoder", back_right_wheel.getCurrentPosition());
            telemetry.addData("Left Encoder", front_right_wheel.getCurrentPosition());
            telemetry.addData("Back Encoder", front_left_wheel.getCurrentPosition());
            telemetry.addData("Drive", trueDrive);
            telemetry.addData("Strafe", trueStrafe);
            telemetry.addData("Rotate", trueRotate);
            telemetry.update();
            */
        }

        public void SetIMUMotors(double drive, double strafe, double rotate, double IMU_zAngle) {
            IMUDrive = strafe*Math.sin(-IMU_zAngle)+drive*Math.sin((Math.PI/2)-IMU_zAngle);
            IMUStrafe = strafe*Math.cos(-IMU_zAngle)+drive*Math.cos((Math.PI/2)-IMU_zAngle);
            this.SetMotors(IMUDrive,IMUStrafe,rotate);
        }

        public void SetMotors(double drive, double strafe, double rotate) {
            //this.frontLeft = drive - strafe - rotate;
            //this.backLeft = -drive - strafe + rotate;
            //this.frontRight = drive + strafe + rotate;
            //this.backRight = drive - strafe + rotate;
            this.frontLeft = drive + strafe - rotate;
            this.backLeft = -drive + strafe + rotate;
            this.frontRight = drive - strafe + rotate;
            this.backRight = drive + strafe + rotate;
        }


        public void Drive() {
            front_right_wheel.setPower(this.frontRight*(1/1));
            front_left_wheel.setPower(this.frontLeft*(1/1));
            back_left_wheel.setPower(this.backLeft*(1/1));
            back_right_wheel.setPower(this.backRight*(1/1));
        }
    }

    public enum OperState {
        NORMALDRIVE,
        NORMALROTATE,
        FORWARD,
        LATERALMOVEMENT,
        SETMOVEMENTDISTANCE,
        SETMOTORMULTIPLE,
        AUTONOMOUSTEST,
        FULLDRIVE,
        ROTATE360,
        ABSOLUTEDRIVE
    }
/*
    @Override
    public void runOpMode(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");

        front_left_wheel = hardwareMap.get(DcMotor.class, "front left wheel");
        front_right_wheel = hardwareMap.get(DcMotor.class, "front right wheel");
        back_left_wheel = hardwareMap.get(DcMotor.class, "back left wheel");
        back_right_wheel = hardwareMap.get(DcMotor.class, "back right wheel");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();  //in wrong spot--where is better?
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu.initialize(parameters);

        waitForStart();

        double drive;
        double strafe;
        double rotate;
        double movementLength = 0;
        double forwardLength;
        double lateralMovement;
        double increaseIntensity = 5;
        boolean upWait = false;
        boolean downWait = false;
        boolean rightWait = false;
        boolean leftWait = false;
        double drivePreset = 0;
        double increaseDecrease = 1;
        boolean aWait = false;
        double autonomousTestStep = 0;
        double rotationGoal = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;;
        ChassisMovementCode.Chassis chassis = new ChassisMovementCode.Chassis();
        ChassisMovementCode.OperState driveOpState = ChassisMovementCode.OperState.NORMALDRIVE;

        while (opModeIsActive()) {

            double zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
            double yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).secondAngle;
            double xAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).thirdAngle;

            switch (driveOpState) {
                case NORMALDRIVE:
                    drive = -this.gamepad1.left_stick_y;
                    strafe = -this.gamepad1.left_stick_x;
                    if ((Math.abs(zAngle - rotationGoal) >= 2)) {
                        rotate = chassis.CorrectRotation(zAngle,rotationGoal);
                    }
                    else {
                        rotate = 0;
                    }

                    chassis.SetMotors (drive, strafe, rotate);
                    chassis.Drive();


                    chassis.Encoders();
                    chassis.SetAxisMovement();



                    if (this.gamepad1.left_trigger != 0) {
                        chassis.ZeroEncoders();
                    }

                    if (this.gamepad1.right_trigger != 0) {
                        driveOpState = ChassisMovementCode.OperState.NORMALROTATE;
                    }

                    if (this.gamepad1.a) {
                        drivePreset = chassis.trueDrive + movementLength;
                        rotationGoal = zAngle;
                        driveOpState = ChassisMovementCode.OperState.FORWARD;
                    }

                    if (this.gamepad1.b) {
                        drivePreset = chassis.trueStrafe + movementLength;
                        rotationGoal = zAngle;
                        driveOpState = ChassisMovementCode.OperState.LATERALMOVEMENT;
                    }

                    if (this.gamepad1.y) {
                        driveOpState = ChassisMovementCode.OperState.SETMOVEMENTDISTANCE;
                    }

                    if (this.gamepad1.x) {
                        driveOpState = ChassisMovementCode.OperState.SETMOTORMULTIPLE;
                    }

                    break;

                case NORMALROTATE:
                    if (this.gamepad1.right_trigger != 0) {
                        rotate = -this.gamepad1.right_stick_x;
                        drive = 0;
                        strafe = 0;

                        chassis.SetMotors(drive, strafe, rotate);
                        chassis.Drive();

                        chassis.SetAxisMovement();

                        rotationGoal = zAngle;

                        if (this.gamepad1.left_trigger != 0) {
                            chassis.ZeroEncoders();
                        }
                        telemetry.addData("IMU rotation", this.imu.getAngularOrientation());
                        telemetry.update();
                    }
                    else {
                        driveOpState = ChassisMovementCode.OperState.NORMALDRIVE;
                    }
                    break;

                case FORWARD:

                    chassis.SetAxisMovement();
                    chassis.Encoders();
                    chassis.ForwardAndBackward(drivePreset);

                    if (this.gamepad1.right_trigger != 0) {
                        driveOpState = ChassisMovementCode.OperState.NORMALDRIVE;
                    }

                    if ((Math.abs(zAngle - rotationGoal) >= 2)) {
                        chassis.SetMotors(0,0,chassis.CorrectRotation(zAngle,rotationGoal));
                        chassis.Drive();
                    }

                    if (Math.abs(drivePreset - chassis.trueDrive) <= 0.2) {
                        front_left_wheel.setPower(0.01);
                        front_right_wheel.setPower(0.01);
                        back_right_wheel.setPower(0.01);
                        back_left_wheel.setPower(0.01);
                        driveOpState = ChassisMovementCode.OperState.NORMALDRIVE;
                    }


                    break;

                case LATERALMOVEMENT:

                    chassis.SetAxisMovement();
                    chassis.Encoders();
                    chassis.LeftAndRight(drivePreset);

                    if (this.gamepad1.right_trigger != 0) {
                        driveOpState = ChassisMovementCode.OperState.NORMALDRIVE;
                    }

                    if ((Math.abs(zAngle - rotationGoal) >= 2)) {
                        chassis.SetMotors(0,0,chassis.CorrectRotation(zAngle,rotationGoal));
                        chassis.Drive();
                    }


                    if (Math.abs(drivePreset - chassis.trueStrafe) <= 0.2) {
                        front_left_wheel.setPower(0.01);
                        front_right_wheel.setPower(0.01);
                        back_right_wheel.setPower(0.01);
                        back_left_wheel.setPower(0.01);
                        driveOpState = ChassisMovementCode.OperState.NORMALDRIVE;
                    }

                    break;
                case SETMOVEMENTDISTANCE:
                    telemetry.addLine("Press up and down on the d-pad to increase movement per press");
                    telemetry.addLine("Press right and left on the d-pad to increase or decrease the increased amount added");
                    telemetry.addData("Current movement per press", movementLength);
                    telemetry.addData("Amount increased per increase", increaseIntensity);
                    telemetry.update();
                    if ((upWait) & (!this.gamepad1.dpad_up)) {
                        movementLength = movementLength + increaseIntensity;
                        upWait = false;
                    }
                    if ((!this.gamepad1.dpad_down) & (downWait)) {
                        movementLength = movementLength - increaseIntensity;
                        downWait = false;
                    }
                    if ((!this.gamepad1.dpad_right) & (rightWait)) {
                        increaseIntensity = increaseIntensity + 1;
                        rightWait = false;
                    }
                    if ((!this.gamepad1.dpad_left) & (leftWait)) {
                        increaseIntensity = increaseIntensity - 1;
                        leftWait = false;
                    }

                    if (this.gamepad1.dpad_up) {
                        upWait = true;
                    }
                    if (this.gamepad1.dpad_down) {
                        downWait = true;
                    }
                    if (this.gamepad1.dpad_right) {
                        rightWait =true;
                    }
                    if (this.gamepad1.dpad_left) {
                        leftWait = true;
                    }

                    if (this.gamepad1.left_trigger != 0) {
                        driveOpState = ChassisMovementCode.OperState.NORMALDRIVE;
                    }

                    break;

                case SETMOTORMULTIPLE:
                    telemetry.addLine("Press A to change increase/decrease");
                    if (increaseDecrease == 1) {
                        telemetry.addLine("INCREASING");
                    }
                    if (increaseDecrease == -1) {
                        telemetry.addLine("DECREASING");
                    }
                    telemetry.addLine("Press right dpad to change FR wheel multiplier");
                    telemetry.addData("FR wheel multiplier: ", chassis.frontRightMultiplier);
                    telemetry.addLine("Press right dpad to change FL wheel multiplier");
                    telemetry.addData("FL wheel multiplier: ", chassis.frontLeftMultiplier);
                    telemetry.addLine("Press right dpad to change BR wheel multiplier");
                    telemetry.addData("BR wheel multiplier: ", chassis.backRightMultiplier);
                    telemetry.addLine("Press right dpad to change BL wheel multiplier");
                    telemetry.addData("BL wheel multiplier: ", chassis.backLeftMultiplier);
                    telemetry.update();

                    if ((increaseDecrease == 1) & (!this.gamepad1.a) & (aWait)) {
                        increaseDecrease = -1;
                        aWait = false;
                    }
                    if ((increaseDecrease == -1) & (!this.gamepad1.a) & (aWait)) {
                        increaseDecrease = 1;
                        aWait = false;
                    }



                    if ((upWait) & (!this.gamepad1.dpad_up)) {
                        chassis.frontLeftMultiplier = chassis.frontLeftMultiplier + (0.01 * increaseDecrease);
                        upWait = false;
                    }
                    if ((!this.gamepad1.dpad_down) & (downWait)) {
                        chassis.backRightMultiplier = chassis.backRightMultiplier + (0.01 * increaseDecrease);
                        downWait = false;
                    }
                    if ((!this.gamepad1.dpad_right) & (rightWait)) {
                        chassis.frontRightMultiplier = chassis.frontRightMultiplier + (0.01 * increaseDecrease);
                        rightWait = false;
                    }
                    if ((!this.gamepad1.dpad_left) & (leftWait)) {
                        chassis.backLeftMultiplier = chassis.backLeftMultiplier + (0.01 * increaseDecrease);
                        leftWait = false;
                    }

                    if (this.gamepad1.dpad_up) {
                        upWait = true;
                    }
                    if (this.gamepad1.dpad_down) {
                        downWait = true;
                    }
                    if (this.gamepad1.dpad_right) {
                        rightWait =true;
                    }
                    if (this.gamepad1.dpad_left) {
                        leftWait = true;
                    }
                    if (this.gamepad1.a) {
                        aWait = true;
                    }

                    if (this.gamepad1.left_trigger != 0) {
                        driveOpState = ChassisMovementCode.OperState.NORMALDRIVE;
                    }

                    break;


                case AUTONOMOUSTEST:
                    if (autonomousTestStep == 0) {
                        drivePreset = chassis.trueDrive + 40;
                        rotationGoal = zAngle;
                        autonomousTestStep = 1;
                        if (this.gamepad1.left_trigger != 0) {
                            driveOpState = ChassisMovementCode.OperState.NORMALDRIVE;
                        }
                    }
                    if (autonomousTestStep == 1) {
                        chassis.SetAxisMovement();
                        chassis.Encoders();
                        chassis.ForwardAndBackward(drivePreset);

                        if (this.gamepad1.left_trigger != 0) {
                            driveOpState = ChassisMovementCode.OperState.NORMALDRIVE;
                        }

                        if ((Math.abs(zAngle - rotationGoal) >= 2)) {
                            chassis.SetMotors(0, 0, chassis.CorrectRotation(zAngle, rotationGoal));
                            chassis.Drive();
                        }

                        if (Math.abs(drivePreset - chassis.trueDrive) <= 0.2) {
                            front_left_wheel.setPower(0.01);
                            front_right_wheel.setPower(0.01);
                            back_right_wheel.setPower(0.01);
                            back_left_wheel.setPower(0.01);
                            autonomousTestStep = 2;
                        }
                    }


                    break;


                default :
                    break;
            }
        }
    */
}

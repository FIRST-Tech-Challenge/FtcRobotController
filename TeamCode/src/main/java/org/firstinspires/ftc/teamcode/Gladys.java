package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.api.movement.old.ChassisMovementCode;

@TeleOp
//Merging, lift, and part of intake done by nahtE, the rest done by lahiN.
//back right front right front left back left
public class Gladys extends LinearOpMode {
    
    private double LeftStickValue;
    private double RightStickValue;
    public DcMotor IntakeMotor;
    public DcMotor IntakeMotor2;
    private Blinker Control_Hub;
    private Blinker expansion_Hub_2;
    ElapsedTime mytimer = new ElapsedTime();
    ElapsedTime debugTimer = new ElapsedTime();

    enum OperState {
        DEBUGSELECT,
        DEBUGONE
    }
    
    @Override
    public void runOpMode() {
        LauncherCode.Launcher launcher = new LauncherCode.Launcher();
        LauncherCode.LauncherStates launchStates = LauncherCode.LauncherStates.Start;
        LifterCode.Lifter lift = new LifterCode.Lifter();

        ChassisMovementCode.Chassis chassis = new ChassisMovementCode.Chassis();
        ChassisMovementCode.OperState driveOpState = ChassisMovementCode.OperState.NORMALDRIVE;
        Gladys.OperState debugOpState = Gladys.OperState.DEBUGSELECT;

        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        lift.LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");

        launcher.LaunchMotor = hardwareMap.get(DcMotor.class, "LaunchMotor");
        launcher.LaunchServo = hardwareMap.get(Servo.class, "LaunchServo");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        IntakeMotor2 = hardwareMap.get(DcMotor.class, "IntakeMotor2");
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
        chassis.imu.initialize(parameters); double drive;
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
        double rotationGoal = chassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.RADIANS).firstAngle;
        double banana2 = -1;

        //double timerStopTime = 0;
        waitForStart();

        while (opModeIsActive()) {
            this.LeftStickValue = -gamepad2.left_stick_y;
            this.RightStickValue = -gamepad2.right_stick_y;
            lift.MoveLift(this.LeftStickValue);

            //telemetry.addData("testing LauncherOn:", launcher.launcherOn);
            //telemetry.addData("Lift Power", lift.LiftPower);
            //telemetry.addData("Fork Power", lift.ForkPower);
            telemetry.update();
            chassis.SetRotation(chassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
            double zAngle = chassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
            double yAngle = chassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).secondAngle;
            double xAngle = chassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).thirdAngle;
            switch (debugOpState) {
                case DEBUGSELECT:
                    //telemetry.addData("Timer Stop Time: ", timerStopTime);
                    telemetry.update();
                    /*
                    if (this.gamepad2.right_trigger != 0) {
                        debugOpState = TwentyTwentyOneOpModeCode.OperState.DEBUGONE;
                        debugTimer.reset();
                    }
                    */

                    break;
                /*
                case DEBUGONE:
                    if (this.gamepad2.right_trigger != 0) {

                    }
                    else {

                        timerStopTime = debugTimer.seconds();
                        debugOpState = TwentyTwentyOneOpModeCode.OperState.DEBUGSELECT;
                    }

                    break;
                 */
            }

            switch (launchStates) {

                case Start:
                    if (this.gamepad2.a) {
                        launchStates = LauncherCode.LauncherStates.ButtonPushed;
                    }

                    if (this.gamepad2.b) {
                        launchStates = LauncherCode.LauncherStates.Pressed;
                    }
                    break;
                case Pressed:
                    if (!this.gamepad2.b) {
                        launchStates = LauncherCode.LauncherStates.firsttimer;

                    }
                    break;
                case firsttimer:
                    mytimer.reset();
                    launchStates = LauncherCode.LauncherStates.Load;
                    break;

                case Load:
                    launcher.Shoot();
                    if (mytimer.time() >= 0.15) {
                        launchStates = LauncherCode.LauncherStates.secondtimer;
                    }
                    break;

                case secondtimer:
                    mytimer.reset();
                    launchStates = LauncherCode.LauncherStates.ResetPosition;
                    break;

                case ResetPosition:
                    launcher.Reload();
                    if (mytimer.time() >= 0.15) {
                        launchStates = LauncherCode.LauncherStates.Start;
                    }
                    break;

                case ButtonPushed:
                    if (!this.gamepad2.a) {
                        launchStates = LauncherCode.LauncherStates.ToggleLauncher;
                    }
                    break;

                case ToggleLauncher:
                    launcher.LauncherToggle();
                    launchStates = LauncherCode.LauncherStates.Start;
                    break;
            }
            launcher.LauncherRun(1);
            switch (driveOpState) {
                case NORMALDRIVE:

                    drive = -this.gamepad1.left_stick_y;
                    strafe = -this.gamepad1.left_stick_x;


                    rotate = 0;


                    chassis.SetIMUMotors(drive,strafe,rotate,chassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.RADIANS).firstAngle);
                    chassis.Drive();


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

                        chassis.ZeroEncoders();
                        driveOpState = ChassisMovementCode.OperState.FULLDRIVE;
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
                        telemetry.addData("IMU rotation", chassis.imu.getAngularOrientation());
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
                        //chassis.SetMotors(0,0,chassis.CorrectRotation(zAngle,rotationGoal));
                        chassis.Drive();
                    }

                    if (Math.abs(drivePreset - chassis.trueDrive) <= 0.2) {
                        chassis.front_left_wheel.setPower(0.01);
                        chassis.front_right_wheel.setPower(0.01);
                        chassis.back_right_wheel.setPower(0.01);
                        chassis.back_left_wheel.setPower(0.01);
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
                        //chassis.SetMotors(0,0,chassis.CorrectRotation(zAngle,rotationGoal));
                        chassis.Drive();
                    }


                    if (Math.abs(drivePreset - chassis.trueStrafe) <= 0.2) {
                        chassis.front_left_wheel.setPower(0.01);
                        chassis.front_right_wheel.setPower(0.01);
                        chassis.back_right_wheel.setPower(0.01);
                        chassis.back_left_wheel.setPower(0.01);
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






                case FULLDRIVE:

                    telemetry.addData("X", chassis.trueX);
                    telemetry.addData("Y",chassis.trueY);
                    telemetry.addData("Preset X", chassis.presetX);
                    telemetry.addData("Preset Y", chassis.presetY);
                    telemetry.addData("True Drive", chassis.trueDrive);
                    telemetry.addData("True Strafe", chassis.trueStrafe);
                    telemetry.addData("true rotate", chassis.trueRotate);
                    telemetry.addData("clear drive", chassis.clearDrive);
                    telemetry.addData("clear strafe", chassis.clearStrafe);
                    telemetry.addData("clear rotate", (chassis.clearRotate/chassis.tau));
                    telemetry.addData("total rotate", (chassis.trueRotate+chassis.clearRotate));


                    chassis.SetAxisMovement();
                    drive = -this.gamepad1.left_stick_y;
                    strafe = this.gamepad1.left_stick_x;
                    rotate = -this.gamepad1.right_stick_x;
                    telemetry.addData("drive",drive);
                    telemetry.addData("strafe",strafe);
                    telemetry.addData("rotate",rotate);

                    chassis.SetMotors (drive, strafe, rotate);

                    chassis.Drive();
                    chassis.SetTrueAxis();
                    break;

                default :
                    break;
            }
        }
    }
}


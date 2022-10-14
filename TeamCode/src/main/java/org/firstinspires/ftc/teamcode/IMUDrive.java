package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="IMUDrive",group="73")
public class IMUDrive extends OpMode {


    // These values affect joystick sensitivity of the arm.
    public double onward                 = 0;
    public DcMotor leftDriveFront        = null;
    public DcMotor rightDriveFront       = null;
    public DcMotor leftDriveBack         = null;
    public DcMotor rightDriveBack        = null;
    public GyroSensor gyro               = null;
    public double leftPowerFront         = 1.0;
    public double rightPowerFront        = 1.0;
    public double rightPowerBack         = 1.0;
    public double leftPowerBack          = 1.0;
    public double drive                  = 0.0;
    public double turn                   = 0.0;
    public double strafe                 = 0.0;
    public double driveAngleOffSet       = 0.0;
    public double speedFactor            = 1.0;
    private CompassSensor compass        = null;

    private ElapsedTime runtime = new ElapsedTime();



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        compass = hardwareMap.get(CompassSensor.class,"compass");
        compass.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
        leftDriveFront = hardwareMap.get(DcMotor.class, "lf");
        rightDriveFront = hardwareMap.get(DcMotor.class, "rf");
        leftDriveBack = hardwareMap.get(DcMotor.class, "lb");
        rightDriveBack = hardwareMap.get(DcMotor.class, "rb");
        // Set Motor Direction
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);


        // Run Without Encoders
        leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Brake when power set to Zero
        leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {


    }

    public void handleCompass() {
        if(gamepad1.a && gamepad1.y){
            onward = gyro.getHeading();
        }
        driveAngleOffSet = gyro.getHeading() - onward;

    }
    public void handleDriveControls() {
        // Get the game pad control values for this loop iteration
        drive = -gamepad1.left_stick_y - gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = 0;
        if (gamepad1.dpad_left)
            strafe = -1;
        else if (gamepad1.dpad_right)
            strafe = 1;
        else {
            strafe = gamepad1.left_stick_x;
        }
        speedFactor = 1.0 - (gamepad1.left_trigger * .5);
    }

    public void handleDriveMotors() {
        leftPowerFront = (drive + turn + strafe) * speedFactor;
        rightPowerFront = (drive - turn - strafe) * speedFactor;
        leftPowerBack = (drive + turn - strafe) * speedFactor;
        rightPowerBack = (drive - turn +strafe) * speedFactor;

        leftDriveFront.setPower(leftPowerFront);
        rightDriveFront.setPower(rightPowerFront);
        leftDriveBack.setPower(leftPowerBack);
        rightDriveBack.setPower(rightPowerBack);

        telemetry.addData("Motors", "lf(%.2f), rf(%.2f), lb(%.2f), rb(%.2f)", leftPowerFront, rightPowerFront, leftPowerBack, rightPowerBack);
        telemetry.addData("Speed control", speedFactor);

    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        double compassDegrees        = compass.getDirection();
        this.handleCompass();
        this.handleDriveControls();

        double od = this.drive;
        double os = this.strafe;

        double sin = Math.sin(driveAngleOffSet * (Math.PI / 180.0));
        double cos = Math.cos(driveAngleOffSet * (Math.PI / 180.0));

        this.drive  = cos * od  + sin * os;
        this.strafe =  -sin * od + cos * os ;

        this.handleDriveMotors();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("sin", sin);
        telemetry.addData("cos", cos);
        telemetry.addData("heading, me mateys", driveAngleOffSet);
        telemetry.addData("compass", compass.getDirection());
        telemetry.addData("comparison C vs I", compassDegrees- driveAngleOffSet);
        telemetry.addData("gyro heading", gyro.getHeading());
    }
}


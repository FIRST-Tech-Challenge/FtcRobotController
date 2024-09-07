package vision;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//so far the newest file as of Feb 5 2024
@TeleOp
public class tbotmovement extends LinearOpMode {

    int button2X = 0;
    int button2A = 0;
    int button2B = 0;
    int button2Y = 0;
    int buttonA = 0;
    int buttonB = 0;
    int buttonX = 0;
    int buttonY = 0;

    boolean but2Acheck = false;
    boolean but2Ycheck = false;
    boolean but2Xcheck = false;
    boolean but2Bcheck = false;

    boolean butAcheck = false;
    boolean butYcheck = false;
    boolean butXcheck = false;
    boolean butBcheck = false;

    boolean directionChange= false;

    double prevtime;

    static double dir;
    static double mag;
    static double pi = Math.PI;

    double offset = 0;
    ElapsedTime timer = new ElapsedTime();

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double angle;

    @Override
    public void runOpMode() {

        // ----------------------Set Up------------------------------------------------
        // Moving
        //DcMotor FL = hardwareMap.get(DcMotor.class, "motorFrontLeft"); // Expansion hub 3
        DcMotor BL = hardwareMap.get(DcMotor.class, "leftMotor"); // Expansion hub 2
        //DcMotor FR = hardwareMap.get(DcMotor.class, "motorFrontRight"); // Control hub 3
        DcMotor BR = hardwareMap.get(DcMotor.class, "rightMotor"); // Control hub 2

        //DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake"); // Expansion hub 1
        //DcMotor Sliders = hardwareMap.get(DcMotor.class, "Sliders"); // Control hub 0
        //DcMotor Climbing1 = hardwareMap.get(DcMotor.class, "Climbing1"); // left Control hub 1
        //DcMotor Climbing2 = hardwareMap.get(DcMotor.class, "Climbing2");// right Expansion hub 0

        //CRServo Pinball1 = hardwareMap.get(CRServo.class, "pinball1"); // left Control hub 1
        //CRServo Pinball2 = hardwareMap.get(CRServo.class, "pinball2");// right Expansion hub 0
        //Servo Wrist = hardwareMap.get(Servo.class, "Wrist"); // control hub servo port 5
        //Servo Launcher = hardwareMap.get(Servo.class, "Launcher"); // control hub servo port 4

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class,"imu");

        imu.initialize(parameters);

        //Wrist.setPosition(0.771);
        //Launcher.setPosition(0);

        /*
         * prevtime = getRuntime();
         * if (getRuntime() - prevtime > 5000)
         */

        //Climbing1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Climbing2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Sliders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Climbing1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Climbing2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Sliders.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reverse right side motors
        //FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        //Sliders.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested())
            return;

        // --------------------------------Mode------------------------------------------------------
        while (opModeIsActive()) {
            // ------------------DRIVE TRAIN---------------------------------
            // Driving

            offset = 0;

            //imu increases when turning left and decreases when turning right

            dir = Math.atan2(-gamepad1.right_stick_y, 0)-offset;
            mag = Math.sqrt(Math.pow(0, 2) + Math.pow(gamepad1.right_stick_y, 2));
            mag *= Math.sqrt(2);
            if (mag > Math.sqrt(2))
                mag = Math.sqrt(2);

            if (gamepad1.right_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.left_stick_x != 0){
                //FR.setPower((Math.sin(dir-(pi/4))*mag) - gamepad1.left_stick_x);
                //FL.setPower((Math.sin(dir+(pi/4))*mag) + gamepad1.left_stick_x);
                BR.setPower((Math.sin(dir+(pi/4))*mag) - gamepad1.left_stick_x);
                BL.setPower((Math.sin(dir-(pi/4))*mag) + gamepad1.left_stick_x);
            } else {
                //FL.setPower(0);
                BL.setPower(0);
                //FR.setPower(0);
                BR.setPower(0);
            }

            telemetry.addData("dir", dir);
            telemetry.addData("offset", offset);
            //telemetry.addData("FR Power", FR.getPower());
            //telemetry.addData("FL Power", FL.getPower());
            telemetry.addData("BR Power", BR.getPower());
            telemetry.addData("BL Power", BL.getPower());
            telemetry.addData("Lx", gamepad1.left_stick_x);
            telemetry.addData("Ly", gamepad1.left_stick_y);
            telemetry.addData("Rx", gamepad1.right_stick_x);
            telemetry.addData("Ry", gamepad1.right_stick_y);


            //telemetry.addData("Slider position", Sliders.getCurrentPosition());
            //telemetry.addData("Slider power", Sliders.getPower());
            //telemetry.addData("Climbing1", Climbing1.getPower());
            //telemetry.addData("Climbing2", Climbing2.getPower());

            //telemetry.addData("Wrist Position", Wrist.getController().getServoPosition(5));
            //telemetry.addData("Wrist Position", Wrist.getPosition()); // whichever one of these 2 works better
            //telemetry.addData("launcher Position", Launcher.getController().getServoPosition(5));
            //telemetry.addData("launcher Position", Launcher.getPosition());
            //telemetry.addData("pinball1 Position", Pinball1.getController());
            //,telemetry.addData("pinball2 Position", Pinball2.getController());

            telemetry.update();
        }
    }
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        angle += deltaAngle;

        lastAngles = angles;

        return angle;
    }
}

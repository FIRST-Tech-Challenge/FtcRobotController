package teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp(name="Mecanum TeleOp Toggle", group="Competition")
public class MovementCode extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private BNO055IMU imu;
    private boolean fieldCentric = false; // start robot-centric

    @Override
    public void runOpMode() throws InterruptedException {
        // Map motors from Robot Configuration
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left");
        backRight  = hardwareMap.get(DcMotor.class, "back_right");

        // Reverse left side to ensure forward is forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS; // radians for trig
        imu.initialize(parameters);

        telemetry.addLine("IMU calibrating...");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addLine("IMU Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Toggle between robot-centric and field-centric when pressing A
            if (gamepad1.a) {
                fieldCentric = !fieldCentric;
                sleep(300); // debounce
            }

            // Joystick controls
            double y  = -gamepad1.left_stick_y;     // Forward/Backward
            double x  = gamepad1.left_stick_x * 1.1; // Strafe (corrected)
            double rx = gamepad1.right_stick_x;    // Rotation

            if (fieldCentric) {
                double heading = imu.getAngularOrientation().firstAngle;
                driveFieldCentric(x, y, rx, heading);
            } else {
                driveRobotCentric(x, y, rx);
            }

            telemetry.addData("Mode", fieldCentric ? "Field Centric" : "Robot Centric");
            telemetry.update();
        }
    }

    /** Robot-centric mecanum drive */
    private void driveRobotCentric(double x, double y, double rx) {
        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        setMotorPowers(fl, fr, bl, br);
    }

    /** Field-centric mecanum drive (joystick rotated by IMU heading) */
    private void driveFieldCentric(double x, double y, double rx, double heading) {
        // Rotate joystick vector by -heading
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double fl = rotY + rotX + rx;
        double bl = rotY - rotX + rx;
        double fr = rotY - rotX - rx;
        double br = rotY + rotX - rx;

        setMotorPowers(fl, fr, bl, br);
    }

    /** Normalize and set motor powers */
    private void setMotorPowers(double fl, double fr, double bl, double br) {
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }
}

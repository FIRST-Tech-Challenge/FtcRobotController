package org.firstinspires.ftc.teamcode.robotSubSystems.Drivetrain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.Utils.Vector;
import org.firstinspires.ftc.teamcode.Utils.Angle;
import org.firstinspires.ftc.teamcode.Utils.PID;

public class Drivetrain {

    private static final DcMotor[] dtMotors = new DcMotor[4];

    public static void init(HardwareMap hardwareMap) {
        dtMotors[0] = hardwareMap.get(DcMotor.class, "lf");
        dtMotors[1] = hardwareMap.get(DcMotor.class, "lb");
        dtMotors[2] = hardwareMap.get(DcMotor.class, "rf");
        dtMotors[3] = hardwareMap.get(DcMotor.class, "rb");

        for (final DcMotor dtMotors : dtMotors) {
            dtMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //todo: set reverse directions
        dtMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        resetEncoders();
    }

    public static void operate(Vector vector, double rotation) {
        drive(
                slowedVec(
                        fieldCentric(vector),
                        0, //Elevator.getElevatorPos(),
                        1, //ElevatorConstance.level3Pos,
                        0.3
                ),
                rotation
        );
    }

    private static Vector slowedVec(Vector vector, double current, double highest, double speed) {
        //0.32
        return vector.scale(Math.max(speed, (highest - current) / highest));
    }

    private static Vector fieldCentric(Vector gamepad) {
        gamepad = gamepad.rotate(-Math.toRadians(Angle.wrapAngle0_360(Gyro.getAngle())));
        return gamepad;
    }

    public static void drive(Vector vector, double rotation) {
        dtMotors[0].setPower(Math.signum(vector.y + vector.x + rotation) * Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed, Math.abs(vector.y + vector.x + rotation))));
        dtMotors[1].setPower(Math.signum(vector.y - vector.x + rotation) * Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed, Math.abs(vector.y - vector.x + rotation))));
        dtMotors[2].setPower(Math.signum(vector.y - vector.x - rotation) * Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed, Math.abs(vector.y - vector.x - rotation))));
        dtMotors[3].setPower(Math.signum(vector.y + vector.x - rotation) * Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed, Math.abs(vector.y + vector.x - rotation))));
    }
    public static void driveByTime(double power) {
        dtMotors[0].setPower(-power);
        dtMotors[1].setPower(-power);
        dtMotors[2].setPower(-power);
        dtMotors[3].setPower(-power);
    }


    public static double getEncoderPos() {
        return dtMotors[3].getCurrentPosition();
    }

    public static void resetEncoders() {
        dtMotors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dtMotors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public static void breakMotors() {
        for (DcMotor motor : dtMotors) {
            motor.setPower(0);
        }
    }


    public static double cmToTicks(double cm) {
        return cm * 65.19;
    }
//    public double ticksToCm(double ticks) {
//        return ticks * DrivetrainConstants.ticksToCM;
//    }
}
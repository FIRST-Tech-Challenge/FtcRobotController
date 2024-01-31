package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="AutonomoAzulD" , group="Linear Opmode")
public class AutonomoAzulD extends LinearOpMode {

    private DcMotor motorEf = null;
    private DcMotor motorEt = null;
    private DcMotor motorDf = null;
    private DcMotor motorDt = null;

    static final double     COUNTS_PER_MOTOR_REV    = 28.0;
    static final double     DRIVE_GEAR_REDUCTION    = 30.4;
    static final double     WHEEL_CIRCUMFERENCE_MM  = 90.0 * 3.14;
    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     COUNTS_PER_MM           = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;

    public static void setupMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void motorMove(DcMotor motor, int distance) {
        int target = (int) (distance * COUNTS_PER_MM);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorEf = hardwareMap.get(DcMotor.class, "ef");
        motorEt = hardwareMap.get(DcMotor.class, "et");
        motorDf = hardwareMap.get(DcMotor.class, "df");
        motorDt = hardwareMap.get(DcMotor.class, "dt");

        setupMotor(motorEf, DcMotorSimple.Direction.FORWARD);
        setupMotor(motorEt, DcMotorSimple.Direction.FORWARD);
        setupMotor(motorDf, DcMotorSimple.Direction.REVERSE);
        setupMotor(motorDt, DcMotorSimple.Direction.REVERSE);

//        motorEf.setDirection(DcMotor.Direction.FORWARD);
//        motorEt.setDirection(DcMotor.Direction.REVERSE);
//        motorDf.setDirection(DcMotor.Direction.FORWARD);
//        motorDt.setDirection(DcMotor.Direction.FORWARD);


//        motorEf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorEt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorDf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorDt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        motorEf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorEt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorDf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorDt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            //andar até passagem
            motorEf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motorEt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motorDf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motorDt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorEf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorEt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motorDf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motorDt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            int esquerdaTarget = (int) (50 * COUNTS_PER_MM);
            //int direitaTarget = (int) (5000 * COUNTS_PER_MM);

            motorEf.setTargetPosition(esquerdaTarget);
//            motorEt.setTargetPosition(esquerdaTarget);
//            motorDf.setTargetPosition(direitaTarget);
//            motorDt.setTargetPosition(direitaTarget);

            motorEf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorEt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorDf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorDt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorEf.setPower(1);
//            motorEt.setPower(0.4);
//            motorDf.setPower(0.4);
//            motorDt.setPower(0.4);


            while (opModeIsActive() && motorEf.isBusy())  { // while (opModeIsActive() && (motorEf.isBusy() && motorDt.isBusy() && motorDf.isBusy() && motorEt.isBusy())) {
//                telemetry.addData("motorDf:", motorDf.getCurrentPosition());
//                telemetry.addData("motorDt:", motorDt.getCurrentPosition());
                telemetry.addData("motorEf:", motorEf.getCurrentPosition());
//                telemetry.addData("motorEt:", motorEt.getCurrentPosition());
//                telemetry.update();
            }

            motorEf.setPower(0);
//            motorEt.setPower(0);
//            motorDf.setPower(0);
//            motorDt.setPower(0);
            sleep(1000);
        }
    }
}

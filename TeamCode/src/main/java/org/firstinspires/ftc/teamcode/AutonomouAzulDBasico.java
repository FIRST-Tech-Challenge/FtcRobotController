package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="AutonomouAzulDBasico" , group="Linear Opmode")
public class AutonomouAzulDBasico extends LinearOpMode {

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
        motor.setPower(0.8);
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorEf = hardwareMap.get(DcMotor.class, "ef");
        motorEt = hardwareMap.get(DcMotor.class, "et");
        motorDf = hardwareMap.get(DcMotor.class, "df");
        motorDt = hardwareMap.get(DcMotor.class, "dt");

        setupMotor(motorEf, DcMotorSimple.Direction.FORWARD);  //PORTA 0 EXPANSION HUB
        setupMotor(motorEt, DcMotorSimple.Direction.FORWARD);  //PORTA 3 EXPANSION HUB
        setupMotor(motorDf, DcMotorSimple.Direction.REVERSE);  //PORTA 3 CONTROL HUB
        setupMotor(motorDt, DcMotorSimple.Direction.REVERSE);  //PORTA 0 CONTROL HUB


        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            //andar até passagem
            motorMove(motorEf, 50);
            motorMove(motorEt, 50);
            motorMove(motorDf, 50);
            motorMove(motorDt, 50);


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

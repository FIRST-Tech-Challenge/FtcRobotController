package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="AutonomoAzulDireito" , group="Linear Opmode")
public class AutonomoAzulDireito extends LinearOpMode {

    class Linear {
        public Linear() {

        }
    }

    class Chassi {
        DcMotor[] motors;
        public Chassi(DcMotor[] motors) {
            this.motors = motors;
        }

        public void moveForward(int distance) {
            setupMotor(this.motors[0], DcMotor.Direction.REVERSE);  // motorEf
            setupMotor(this.motors[1], DcMotor.Direction.FORWARD);  // motorEt
            setupMotor(this.motors[2], DcMotor.Direction.REVERSE);  // motorDf
            setupMotor(this.motors[3], DcMotor.Direction.FORWARD);  // motorDt
            motorMove(this.motors[0], distance);  // motorEf
            motorMove(this.motors[1], distance);  // motorEt
            motorMove(this.motors[2], distance);  // motorDf
            motorMove(this.motors[3], distance);  // motorDt
        }

        public void moveBackward(int distance) {
            setupMotor(this.motors[0], DcMotor.Direction.FORWARD);  // motorEf
            setupMotor(this.motors[1], DcMotor.Direction.REVERSE);  // motorEt
            setupMotor(this.motors[2], DcMotor.Direction.FORWARD);  // motorDf
            setupMotor(this.motors[3], DcMotor.Direction.REVERSE);  // motorDt
            motorMove(this.motors[0], distance);  // motorEf
            motorMove(this.motors[1], distance);  // motorEt
            motorMove(this.motors[2], distance);  // motorDf
            motorMove(this.motors[3], distance);  // motorDt
        }

        public void moveLeft(int distance) {
            setupMotor(this.motors[0], DcMotor.Direction.REVERSE);  // motorEt
            setupMotor(this.motors[1], DcMotor.Direction.FORWARD);  // motorEf
            setupMotor(this.motors[2], DcMotor.Direction.FORWARD);  // motorDt
            setupMotor(this.motors[3], DcMotor.Direction.REVERSE);  // motorDf
            motorMove(this.motors[0], distance);  // motorEf
            motorMove(this.motors[1], distance);  // motorEt
            motorMove(this.motors[2], distance);  // motorDf
            motorMove(this.motors[3], distance);  // motorDt
        }

        public void moveRight(int distance) {
            setupMotor(this.motors[0], DcMotor.Direction.FORWARD);  // motorEf
            setupMotor(this.motors[1], DcMotor.Direction.REVERSE);  // motorEt
            setupMotor(this.motors[2], DcMotor.Direction.REVERSE);  // motorDf
            setupMotor(this.motors[3], DcMotor.Direction.FORWARD);  // motorDt
            motorMove(this.motors[0], distance);  // motorEf
            motorMove(this.motors[1], distance);  // motorEt
            motorMove(this.motors[2], distance);  // motorDf
            motorMove(this.motors[3], distance);  // motorDt
        }

        public void turnLeft(int angle) {
            setupMotor(this.motors[0], DcMotor.Direction.FORWARD);  // motorEf
            setupMotor(this.motors[1], DcMotor.Direction.FORWARD);  // motorEt
            setupMotor(this.motors[2], DcMotor.Direction.FORWARD);  // motorDf
            setupMotor(this.motors[3], DcMotor.Direction.FORWARD);  // motorDt
            // Calcular quanto as rodas têm que se mover para que o Robô atinja certo ângulo
            motorMove(this.motors[0], angle);  // motorEf
            motorMove(this.motors[1], angle);  // motorEt
            motorMove(this.motors[2], angle);  // motorDf
            motorMove(this.motors[3], angle);  // motorDt
        }

        public void turnRight(int angle) {
            setupMotor(this.motors[0], DcMotor.Direction.REVERSE);  // motorEf
            setupMotor(this.motors[1], DcMotor.Direction.REVERSE);  // motorEt
            setupMotor(this.motors[2], DcMotor.Direction.REVERSE);  // motorDf
            setupMotor(this.motors[3], DcMotor.Direction.REVERSE);  // motorDt
            // Calcular quanto as rodas têm que se mover para que o Robô atinja certo ângulo
            motorMove(this.motors[0], angle);  // motorEf
            motorMove(this.motors[1], angle);  // motorEt
            motorMove(this.motors[2], angle);  // motorDf
            motorMove(this.motors[3], angle);  // motorDt
        }
    }

    class TeamRobot {
        Chassi chassi;
        Linear linear;
        public TeamRobot(Chassi chassi, Linear linear) {
            this.chassi = chassi;
            this.linear = linear;
        }

        public void move(String direction, float distance) {
            int length = (int) Math.floor(distance);
            switch (direction) {
                case "forward":
                    this.chassi.moveForward(length);
                    break;
                case "backward":
                    this.chassi.moveBackward(length);
                    break;
                case "left":
                    this.chassi.moveLeft(length);
                    break;
                case "right":
                    this.chassi.moveRight(length);
                    break;
                default:
                    break;
            }
        }

        public void turn(String direction, int angle) {
            switch (direction) {
                case "left":
                    this.chassi.turnLeft(angle);
                    break;
                case "right":
                    this.chassi.turnRight(angle);
                    break;
                default:
                    break;
            }
        }



    }


    private DcMotor motorEf = null;
    private DcMotor motorEt = null;
    private DcMotor motorDf = null;
    private DcMotor motorDt = null;

    static final double     COUNTS_PER_MOTOR_REV    = 28.0;
    static final double     DRIVE_GEAR_REDUCTION    = 30.4;
    static final double     WHEEL_CIRCUMFERENCE_MM  = 90.0 * 3.14;
    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     COUNTS_PER_MM           = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;
    static final int     TATAMI_SIDE_SIZE           = 584;

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

        motorEf = hardwareMap.get(DcMotor.class, "ef");  //PORTA 0 EXPANSION HUB
        motorEt = hardwareMap.get(DcMotor.class, "et");  //PORTA 3 EXPANSION HUB
        motorDf = hardwareMap.get(DcMotor.class, "df");  //PORTA 3 CONTROL HUB
        motorDt = hardwareMap.get(DcMotor.class, "dt");  //PORTA 0 CONTROL HUB

        DcMotor tractionMotors[] = {motorEf, motorEt, motorDf, motorDt};

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            Linear linear = new Linear();
            Chassi chassi = new Chassi(tractionMotors);
            TeamRobot robot = new TeamRobot(chassi, linear);

            //andar até passagem
            robot.move("forward", (TATAMI_SIDE_SIZE * (1/4)));  //mover 1/4 do tatame
            robot.move("left", TATAMI_SIDE_SIZE * 4);


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

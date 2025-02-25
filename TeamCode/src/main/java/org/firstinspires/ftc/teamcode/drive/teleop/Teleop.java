package org.firstinspires.ftc.teamcode.drive.teleop;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Teleop extends OpMode {
    //CHASSI
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor frontRight;

    //COLETA
    Servo linkageRight;
    Servo linkageLeft;
    Servo pulsoRight;
    Servo pulsoLeft;
    Servo garra;
    Servo rotate;

    //ENTREGA
    DcMotor polia;
    Servo bracinho;
    Servo garrinha;
    double ticks = 2550;
    double ticks2 = 1000;
    double ticks3 = 1450;
    double newTarget;
    private static final double COUNTS_PER_MOTOR_REV = 336; // Rev HD Hex Motor
    private static final double WHEEL_DIAMETER_INCHES = 3.77;  // Diameter of the wheel in inches
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (Math.PI * WHEEL_DIAMETER_INCHES);
    private boolean isPoliaMoving = false;
    private long poliaStartTime = 0;


    @Override
    public void init(){
        //CHASSI
        //Nomes dos motores
        backLeft = hardwareMap.dcMotor.get("BL");
        backRight = hardwareMap.dcMotor.get("BR");
        frontLeft = hardwareMap.dcMotor.get("FL");
        frontRight = hardwareMap.dcMotor.get("FR");

        //Ativar os encoders dos motores
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Qunado não tiver energia aplicada ele força uma parada
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reversão de valores
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //COLETA
        //Nomes dos servos
        linkageLeft = hardwareMap.get(Servo.class, "lleft");
        linkageRight = hardwareMap.get(Servo.class, "lright");
        pulsoLeft = hardwareMap.get(Servo.class, "pleft");
        pulsoRight = hardwareMap.get(Servo.class, "pright");
        garra = hardwareMap.get(Servo.class, "garra");
        rotate = hardwareMap.get(Servo.class, "rotate");

        //ENTREGA
        //Nomes dos servos/motor
        bracinho = hardwareMap.get(Servo.class, "turn");
        garrinha = hardwareMap.get(Servo.class, "garrinha");
        polia = hardwareMap.get(DcMotor.class, "polia");
        polia.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        polia.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // LEMBRA DE SETAR AS PRE-POSIÇÕES DOS SERVOS

        telemetry.addData("Status: ", "Pronto");
        telemetry.addLine("É US GURI DE SC PORRA");
        pulsoRight.setPosition(0.9);
        pulsoLeft.setPosition(0);
        linkageRight.setPosition(1);
        linkageLeft.setPosition(0);
        rotate.setPosition(0.65);
        garrinha.setPosition(0.7);
        bracinho.setPosition(0.24);
        garra.setPosition(0.45);
    }
    @Override
    public void loop(){
        //CHASSI
        double forward = gamepad2.left_stick_y;
        double strafe = -gamepad2.left_stick_x;
        double turn = -gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 2);

        frontRight.setPower((forward - strafe - turn) / denominator);
        frontLeft.setPower((forward + strafe + turn) / denominator);
        backLeft.setPower((forward - strafe + turn) / denominator);
        backRight.setPower((forward + strafe - turn) / denominator);

        //COLETA
        if (gamepad1.dpad_up){
            linkageLeft.setPosition(0.55);
            linkageRight.setPosition(0.6);
            sleep(100);
            pulsoRight.setPosition(0.25);
            pulsoLeft.setPosition(0.65);
            garra.setPosition(0.5);
        }
        if (gamepad1.dpad_down){
            garra.setPosition(0.95);
            sleep(200);
            pulsoRight.setPosition(0.85);
            pulsoLeft.setPosition(0.05);
            sleep(500);
            linkageRight.setPosition(1);
            linkageLeft.setPosition(0);
        }
        if (gamepad1.dpad_left){
            linkageLeft.setPosition(0.3);
            linkageRight.setPosition(0.7);
            sleep(100);
            pulsoRight.setPosition(0.25);
            pulsoLeft.setPosition(0.65);
            garra.setPosition(0.5);
        }
        if (gamepad1.x){
            pulsoRight.setPosition(0.9);
            pulsoLeft.setPosition(0.05);
        }
        if (gamepad1.y){
            pulsoRight.setPosition(0.14);
            pulsoLeft.setPosition(0.74);
        }
        if (gamepad1.b){
            garra.setPosition(0.95);
        }
        if(gamepad1.a){
            garra.setPosition(0.45);
        }
        if(gamepad1.right_bumper){
            rotate.setPosition(1);
        }
        if (gamepad1.left_bumper){
            rotate.setPosition(0.65);
        }
        //ENTREGA
        if (gamepad2.dpad_up) {
            poliaBasket(1);
        }
        if (gamepad2.dpad_left){
            poliaClip();
        }
        // HANDLING POLIA TIMING
        if (isPoliaMoving) {
            if (System.currentTimeMillis() - poliaStartTime > 300) {
                garrinha.setPosition(0.6);
            }
            if (System.currentTimeMillis() - poliaStartTime > 1550) {
                polia.setPower(0);
                isPoliaMoving = false; // Reset flag when done
            }
        }
        // OTHER CONTROLS
        if (gamepad2.dpad_down && !isPoliaMoving) {
            bracinho.setPosition(0.25);
            poliaDown();
        }
        if (gamepad2.dpad_right){
            poliaClipar(1);
        }
        if (gamepad2.b){
            garrinha.setPosition(1);
            sleep(300);
            garra.setPosition(0.45);
        }
        if (gamepad2.a) {
            garrinha.setPosition(0.7);
        }
        if (gamepad2.x){
            bracinho.setPosition(0.24);
        }
        if (gamepad2.y){
            bracinho.setPosition(1);
        }
        if (gamepad2.left_bumper){
            bracinho.setPosition(0.8);
        }
    }
    //Funções da polia (cada função é uma altura diferente)
    public void poliaBasket(int turnage) {
        newTarget = ticks / turnage;
        polia.setTargetPosition((int) newTarget);
        polia.setPower(1);
        polia.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(200);
        bracinho.setPosition(0.85);
    }
    public void poliaClip() {
        newTarget = ticks2 / 2;
        polia.setTargetPosition((int) newTarget);
        polia.setPower(1);
        polia.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(600);
        garrinha.setPosition(0.6);
        bracinho.setPosition(0.24);
        sleep(200);
        polia.setTargetPosition(0);
        polia.setPower(1);
        polia.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void poliaClipar(int turnage){
        newTarget = ticks3 / turnage;
        polia.setTargetPosition((int) newTarget);
        polia.setPower(1);
        polia.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bracinho.setPosition(1);
    }
    public void poliaDown() {
        polia.setTargetPosition(0);
        polia.setPower(1);
        polia.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        isPoliaMoving = true; // Flag to indicate polia movement
        poliaStartTime = System.currentTimeMillis(); // Store start time
    }

    private void driveInches(double inches, double speed) {

        int targetPosition = (int) (inches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + targetPosition);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + targetPosition);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + targetPosition);
        backRight.setTargetPosition(backRight.getCurrentPosition() + targetPosition);

        setRunToPosition();

        setPower(speed);

        stopMotors();
        resetEncoders();
    }
    private void setRunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void setPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
    private void stopMotors() {
        setPower(0);
    }
    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

/*
        CONTROLES

START A: JOSÉ - COLETA
- DPAD-UP = linkage pra frente
- DPAD-DOWN = linkage pra trás
- X = pulso desce
- Y = pulso sobe
- A = garra abre
- B = garra fecha
- RIGHT_BUMPER = garra vira pra direita
- LEFT_BUMPER = garra volta pro meio

START B: DAVI - ENTREGA/CHASSI
- DPAD-UP = polia sobe na altura do high basket
- DPAD-LEFT = polia sobe pra tirar o clip da parede
- DPAD-RIGHT = polia sobe na altura pra clipar
- DPAD-DOWN = polia desce
- X = bracinho volta
- Y = bracinho vira
- A = garrinha abre
- B = garrinha fecha
- JOYSTICK_LEFT = Move/Strafe
- JOYSTICK_RIGHT = Turn


CONTROL HUB
Motores:
0 - FL
1 - FR
2 - BR
3 - BL

Servos:
0 - rotate
1 - garra
2 - pulso esquerda
3 - pulso direita
4 - linkage esquerda
5 - linkage direita

EXPANSION HUB
Motores:
0 - Polia
1 - RightOdo
2 - LeftOdo
3 - MidOdo

Servos:
0 - bracinho
1 - garrinha
2 -
3 -
4 -
5 -

 */

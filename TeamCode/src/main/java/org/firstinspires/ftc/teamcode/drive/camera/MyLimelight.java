package org.firstinspires.ftc.teamcode.drive.camera;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous
public class MyLimelight extends LinearOpMode {

    public Limelight3A limelight;
    public IMU imu;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    static final double TICKS_PER_CM = 17.82; // Para o nosso chassi
    static final double TURN_TOLERANCE = 2.0; // graus de margem de erro
    boolean verificacao = false;
    boolean verificacaoY = false;

    DcMotor poliaright;
    DcMotor polialeft;
    Servo Bright;
    Servo Bleft;
    Servo garrinha;
    double ticks = 2750;
    double newTarget;

    Servo rotate;
    Servo garra;
    Servo pleft;
    Servo pright;
    Servo lright;
    Servo lleft;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(myIMUparameters);

        lright = hardwareMap.get(Servo.class, "lright");
        lleft = hardwareMap.get(Servo.class, "lleft");
        rotate = hardwareMap.get(Servo.class, "rotate");
        garra = hardwareMap.get(Servo.class, "garra");
        pleft = hardwareMap.get(Servo.class, "pleft");
        pright = hardwareMap.get(Servo.class, "pright");

        //OUTTAKE
        poliaright = hardwareMap.get(DcMotor.class, "poliaright");
        polialeft = hardwareMap.get(DcMotor.class, "polialeft");
        Bright = hardwareMap.get(Servo.class, "bright");
        Bleft = hardwareMap.get(Servo.class, "bleft");
        garrinha = hardwareMap.get(Servo.class, "garrinha");

        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        poliaright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        polialeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                // First, tell Limelight which way your robot is facing
                double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                limelight.updateRobotOrientation(robotYaw);
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().x * 100;// * 100 para transforma de m pra cm
                    double y = botpose_mt2.getPosition().y * 100;// * 100 para transforma de m pra cm
                    telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                    // Etapas sequenciais usando verificação booleana
                    if (!verificacao) {
                        if (Math.abs(x + 164) > 2) {
                            startAllMotors(0.2);
                        } else {
                            stopAllMotors();
                            verificacao = true;
                        }
                    } else if (!verificacaoY) {
                        if (Math.abs(y + 81) > 2) {
                            strafeAllMotors(0.2);
                        } else {
                            stopAllMotors();
                            verificacaoY = true;
                        }
                    } else {
                        moveCM(10, 0.2);
                        sleep(200);
                        turnToAngle(-45);
                        moveCM(5,0.2);
                        sleep(500);
                        //Entrega();
                        turnToAngle(-10);
                        sleep(100);
                        moveCM(-17,0.2);
                        //Coleta();
                        break;
                    }



                }
            } else {
                telemetry.addData("Limelight", "No Targets");
            }
        }
    }

    private void resetEncoders() {
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void startAllMotors(double power){
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }
    public void strafeAllMotors(double power){
        frontLeft.setPower(power);
        backLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(power);
    }

    private void moveCM(double cm, double power) {
        int ticks = (int) (cm * TICKS_PER_CM);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + ticks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addLine("Movendo para frente...");
            telemetry.update();
        }

        stopAllMotors();
    }
    public void Entrega(){
        viperslide1Up(-1);
        viperslide2Up(1);
        sleep(500);
        Bright.setPosition(0.4);
        Bleft.setPosition(0.6);
        sleep(100);
        viperslide1Down();
        viperslide2Down();
    }
    public void Coleta(){
        lright.setPosition(0.6);
        lleft.setPosition(0.7);
        sleep(200);
        garra.setPosition(0.3);
        pleft.setPosition(0);
        pright.setPosition(1);
        sleep(100);
        garra.setPosition(0.6);
        sleep(200);
        rotate.setPosition(0.7);
        pleft.setPosition(0.8);
        pright.setPosition(0.2);
        lright.setPosition(1);
        lleft.setPosition(0.1);
        sleep(500);
        garra.setPosition(0.3);
    }
    public void viperslide1Up ( int turnage){
        newTarget = ticks / turnage;
        poliaright.setTargetPosition((int) newTarget);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide2Up ( int turnage){
        newTarget = ticks / turnage;
        polialeft.setTargetPosition((int) newTarget);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide1Down () {
        poliaright.setTargetPosition(0);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide2Down () {
        polialeft.setTargetPosition(0);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void strafeCM(double cm, double power) {
        int ticks = (int) (cm * TICKS_PER_CM);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - ticks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addLine("Strafando...");
            telemetry.update();
        }

        stopAllMotors();
    }

    private void turnToAngle(double targetAngle) {
        double currentYaw = getYaw();
        double error = angleDiff(targetAngle, currentYaw);

        // Corrigir modo dos motores
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        while (opModeIsActive() && Math.abs(error) > TURN_TOLERANCE) {
            double power = 0.25 * Math.signum(error);

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            currentYaw = getYaw();
            error = angleDiff(targetAngle, currentYaw);

            telemetry.addData("Yaw", currentYaw);
            telemetry.addData("Target", targetAngle);
            telemetry.update();

        }
        stopAllMotors();
    }

    private double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double angleDiff(double target, double current) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    private void stopAllMotors() {
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setPower(0);
        }
    }
}

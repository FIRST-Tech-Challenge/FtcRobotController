package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shuangren")
public class Shuangren extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotor slideBack, slideBack2, slideRight, slideLeft;
    private Servo frontclawbigPitch, frontclawsmallPitch, frontclaw, frontclawTurn;
    private Servo backclaw, backclawPitch;

    private double frontclaw_OPEN = 0.4;
    private double frontclaw_CLOSE = 0.825;
    private double backclaw_OPEN = 0.4;
    private double backclaw_CLOSE = 0.7;
    private double frontclawbigPitch_GRAB = 0.85;
    private double frontclawbigPitch_PREPARE = 0.6;
    private double frontclawbigPitch_HANDOVER = 0.5;
    private double frontclawsmallPitch_GRAB = 0;
    private double frontclawsmallPitch_HANDOVER = 0.81;
    private double frontclawTurn_LEVEL = 0.8;
    private double backclawPitch_HANDOVER = 0.63;
    private double backclawPitch_RELEASE = 0.01;

    private boolean isFirstA = true;

    @Override
    public void runOpMode() {
        // 初始化硬件
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");
        slideBack2 = hardwareMap.get(DcMotor.class,"slideBacksecond");
        slideBack = hardwareMap.get(DcMotor.class,"slideBack");
        frontclawbigPitch = hardwareMap.get(Servo.class, "frontclawbigPitch");
        frontclawTurn = hardwareMap.get(Servo.class, "frontclawTurn");
        frontclawsmallPitch = hardwareMap.get(Servo.class, "frontclawsmallPitch");
        frontclaw = hardwareMap.get(Servo.class, "frontclaw");
        backclaw = hardwareMap.get(Servo.class, "backclaw");
        backclawPitch = hardwareMap.get(Servo.class, "backclawPitch");



        // 设置电机方向
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setDirection(DcMotor.Direction.REVERSE);
        slideLeft.setDirection(DcMotor.Direction.REVERSE);
        slideBack.setDirection(DcMotor.Direction.FORWARD);
        slideBack2.setDirection(DcMotor.Direction.FORWARD);

        // 刹车
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideBack2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        // 创建并启动底盘控制线程
        Thread driveThread = new Thread(this::driveControl);
        driveThread.start();

        // 主线程用于控制爪子和滑轨
        while (opModeIsActive()) {
            controlClawAndSlides();
        }
    }

    // 底盘控制逻辑（运行在单独线程中）
    private void driveControl() {
        while (opModeIsActive()) {
            double drive = -gamepad2.left_stick_y;
            double strafe = gamepad2.left_stick_x;
            double turn = gamepad2.right_stick_x;

            // 应用二次曲线使控制更加平滑
            drive = Math.signum(drive) * drive * drive * 0.8;
            strafe = Math.signum(strafe) * strafe * strafe * 0.8;
            turn = Math.signum(turn) * turn * turn * 0.8;

            if (Math.abs(drive) < 0.05) drive = 0;
            if (Math.abs(strafe) < 0.05) strafe = 0;
            if (Math.abs(turn) < 0.05) turn = 0;

            double leftFrontPower = drive + strafe + turn;
            double rightFrontPower = drive - strafe - turn;
            double leftBackPower = drive - strafe + turn;
            double rightBackPower = drive + strafe - turn;

            // 电机标准化
            double maxPower = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));
            if (maxPower > 0) {
                leftFrontPower /= maxPower;
                rightFrontPower /= maxPower;
                leftBackPower /= maxPower;
                rightBackPower /= maxPower;
            }

            // 设置底盘电机功率
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);

            // 短暂休眠以避免占用过多CPU资源
            sleep(10);
        }
    }

    // 爪子和滑轨控制逻辑（运行在主线程中）
    private void controlClawAndSlides() {
        // 控制滑轨
        if (gamepad2.left_bumper) {
            frontclaw.setPosition(frontclaw_OPEN);
            frontclawTurn.setPosition(frontclawTurn_LEVEL);
            frontclawsmallPitch.setPosition(frontclawsmallPitch_GRAB);
            frontclawbigPitch.setPosition(frontclawbigPitch_PREPARE);
            setFrontSlidePower(-1);
            sleep(330);
            setFrontSlidePower(0);
        }

        if (gamepad2.right_bumper) {
            frontclawTurn.setPosition(frontclawTurn_LEVEL);
            sleep(200);
            frontclawsmallPitch.setPosition(frontclawsmallPitch_HANDOVER);
            backclaw.setPosition(backclaw_OPEN);
            frontclawbigPitch.setPosition(frontclawbigPitch_HANDOVER);
            backclawPitch.setPosition(backclawPitch_HANDOVER);
            frontclaw.setPosition(frontclaw_CLOSE);
            sleep(400);
            setFrontSlidePower(1);
            sleep(330);
            setFrontSlidePower(0);
            backclaw.setPosition(backclaw_OPEN);
            sleep(50);
            backclaw.setPosition(0.7);
            sleep(150);
            frontclaw.setPosition(frontclaw_OPEN);
        }

        if (gamepad2.a){
            if (isFirstA) {
                backclaw.setPosition(backclaw_OPEN);
                frontclawsmallPitch.setPosition(frontclawsmallPitch_GRAB);
                frontclawbigPitch.setPosition(frontclawbigPitch_GRAB);
                sleep(150);
                frontclaw.setPosition(frontclaw_CLOSE);
                sleep(100);
                frontclawbigPitch.setPosition(frontclawbigPitch_HANDOVER);
                isFirstA = false;
            } else {
                backclaw.setPosition(backclaw_OPEN);
                frontclaw.setPosition(frontclaw_OPEN);
                frontclawsmallPitch.setPosition(frontclawsmallPitch_GRAB);
                frontclawbigPitch.setPosition(frontclawbigPitch_PREPARE);
                isFirstA = true;
            }
        }

        if (gamepad2.dpad_left) {
            frontclawTurn.setPosition(Math.max(frontclawTurn.getPosition() - 0.01, 0.3));
        } else if (gamepad2.dpad_right) {
            frontclawTurn.setPosition(Math.min(frontclawTurn.getPosition() + 0.01, 0.7));
        }

        if (gamepad2.x) {
            backclaw.setPosition(backclaw_CLOSE);
            setBackSlidePower(-1);
            sleep(300);
            backclawPitch.setPosition(backclawPitch_RELEASE);
            sleep(300);
            setBackSlidePower(0);
        }

        if (gamepad2.y) {
            backclaw.setPosition(backclaw_OPEN);
            sleep(300);
            backclawPitch.setPosition(backclawPitch_HANDOVER);
            sleep(300);
            setBackSlidePower(1);
            sleep(600);
            setBackSlidePower(0);
        }


        // 短暂休眠以避免占用过多CPU资源
        sleep(10);
    }

    // 设置前滑轨功率的辅助方法
    private void setFrontSlidePower(double power) {
        slideRight.setPower(-power);
        slideLeft.setPower(-power);
    }
    // 设置后滑轨功率的辅助方法
    private void setBackSlidePower(double power) {
        slideBack.setPower(power);
        slideBack2.setPower(power);
    }
}

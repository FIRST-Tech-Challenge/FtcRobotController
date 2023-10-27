    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.Disabled;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.acmerobotics.dashboard.FtcDashboard;
    import java.util.ArrayList;
    import java.util.Random;
    @Disabled
    @TeleOp(name = "Drive2")

    public class Swaws extends LinearOpMode {


        private DcMotorEx FrontLeftMotor;
        private DcMotorEx BackLeftMotor;
        private DcMotorEx FrontRightMotor;
        private DcMotorEx BackRightMotor;
        private Servo Claw1;
        private Servo Claw2;
        private Servo Claw3;
        private DcMotor Arm1;


        /**
         * This function is executed when this Op Mode is selected from the Driver Station.
         */
        /**
         * The override indicates that the child class is overwriting its base class
         * The public void means it can be accessed but does not return a type which in this instance runs program
         */

        @Override
        public void runOpMode() throws InterruptedException {
            inithardware();
            waitForStart();

            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    driving();
                    telemetry();
                }
            }
        }

        public void inithardware() {
            initmotors();
            initservos();
        }

        public void initmotors() {
            FrontLeftMotor = hardwareMap.get(DcMotorEx.class, "FrontLeftMotor");
            BackLeftMotor = hardwareMap.get(DcMotorEx.class, "BackLeftMotor");
            FrontRightMotor = hardwareMap.get(DcMotorEx.class, "FrontRightMotor");
            BackRightMotor = hardwareMap.get(DcMotorEx.class, "BackRightMotor");
            Arm1 = hardwareMap.get(DcMotor.class, "Arm");
            FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void initservos() {
            Claw1 = hardwareMap.get(Servo.class, "Claw1");
            Claw2 = hardwareMap.get(Servo.class, "Claw2");
            Claw3 = hardwareMap.get(Servo.class, "Claw3");
            Claw2.setDirection(Servo.Direction.REVERSE);
            Claw1.setPosition(0);
            Claw2.setPosition(0);
            Claw3.setPosition(0);

        }

        public void telemetry() {
            telemetry.addData("Powerfl", String.valueOf(FrontLeftMotor.getPower()));
            telemetry.addData("Powerfr", String.valueOf(FrontRightMotor.getPower()));
            telemetry.addData("Powerbl", String.valueOf(BackLeftMotor.getPower()));
            telemetry.addData("Powerbr", String.valueOf(BackRightMotor.getPower()));
            telemetry.addData("Claw1 Position", Claw1.getPosition());
            telemetry.addData("Claw2 Position", Claw2.getPosition());
            telemetry.addData("Claw3 Position", Claw3.getPosition());
            telemetry.addData("ArmPosition", Arm1.getCurrentPosition());
            telemetry.update();

        }

        public void driving() {
            double speed = 1;
            double Arm = gamepad2.left_stick_y;
            double x = gamepad1.left_stick_x * 0.8;
            double y = -gamepad1.left_stick_y * 0.8;
            double turn = gamepad1.right_stick_x * 0.8;
            double Y = gamepad2.right_stick_y;
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);
            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));
            double X = gamepad2.left_stick_x;
            double fl = (power * cos / max + turn);
            double fr = (power * sin / max - turn);
            double bl = (power * sin / max + turn);
            double br = (power * cos / max - turn);
            if ((power + Math.abs(turn)) > 1) {
                fl /= power + Math.abs(turn);
                fr /= power + Math.abs(turn);
                bl /= power + Math.abs(turn);
                br /= power + Math.abs(turn);
            }


            FrontLeftMotor.setPower(fl * speed);
            FrontRightMotor.setPower(fr * speed);
            BackLeftMotor.setPower(bl * speed);
            BackRightMotor.setPower(br * speed);


            Claw1.setPosition(0 + (gamepad2.right_stick_y * 0.5));

            if (gamepad2.left_bumper) {
                Claw3.setPosition(180);
            } else if (gamepad2.right_bumper) {
                Claw3.setPosition(0);
            } else if (gamepad1.dpad_down) {
                speed -= .2;
            } else if (gamepad1.dpad_up) {
                speed += .2;
            } else if (gamepad1.dpad_left) {
                speed = 1;
            }

            Arm1.setPower(Arm);

        }
    }





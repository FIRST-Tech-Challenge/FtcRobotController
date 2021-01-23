package org.firstinspires.ftc.teamcode.mason_wu;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Mecanum TeleOp2", group = "Linear Opmode")
public class Mecanum_TeleOp extends LinearOpMode {

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private DcMotor shooter = null;
    private DcMotor intake = null;
    private Servo spanker = null;
    private DcMotor arm = null;
    private Servo hand = null;

    double rotate = 0;
    double speed = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB  = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        spanker = hardwareMap.get(Servo.class,"spanker");
        arm = hardwareMap.get(DcMotor.class, "arm");
        hand = hardwareMap.get(Servo.class, "hand");

        shooter.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive()) {
            double LFPower;
            double RFPower;
            double LBPower;
            double RBPower;

            double drive = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;



            if(gamepad1.dpad_up){
                speed += 0.05;
                if(speed < 0){
                    speed = 0;
                }
            }
            if(gamepad1.dpad_down){
                speed -= 0.05;
                if(speed < 0){
                    speed = 0;
                }
            }

            LFPower  = Range.clip(speed*(drive + rotate - strafe), -1.0, 1.0) ;
            LBPower  = Range.clip(speed*(drive + rotate + strafe), -1.0, 1.0) ;
            RFPower  = Range.clip(speed*(drive - rotate + strafe), -1.0, 1.0) ;
            RBPower  = Range.clip(speed*(drive - rotate - strafe), -1.0, 1.0) ;

            LF.setPower(LFPower);
            RF.setPower(RFPower);
            LB.setPower(LBPower);
            RB.setPower(RBPower);

            if(gamepad1.right_bumper) {
                shooter.setPower(1.0);
            } else {
                shooter.setPower(0.0);
            }

            if(gamepad1.left_bumper) {
<<<<<<< HEAD
                intake.setPower(0.45);
            } else if(gamepad1.x) {
               intake.setPower(-0.45);
=======
                intake.setPower(0.55);
            } else if(gamepad1.x) {
               intake.setPower(-0.55);
>>>>>>> ddce3dfc3b8bef69065c5da67c154370960da050
            } else {
              intake.setPower(0.0);
            }

            if(gamepad1.right_trigger == 1) {
                spanker.setPosition(0.55);
            } else {
                spanker.setPosition(0.85);
            }

            if(gamepad1.y) {
                arm.setPower(0.8);
            } else {
                arm.setPower(0);
            }

            if(gamepad1.b) {
                arm.setPower(-0.8);
            } else {
                arm.setPower(0);
            }

            if(gamepad1.left_trigger == 1) {
                hand.setPosition(0.9);
            } else {
                hand.setPosition(0);
            }

            telemetry.addData("Front Motors", "LF (%.2f), RF (%.2f)", LFPower, RFPower);
            telemetry.addData("Back Motors", "LB (%.2f), RB (%.2f)", LBPower, RBPower);
            telemetry.update();
        }

    }
}
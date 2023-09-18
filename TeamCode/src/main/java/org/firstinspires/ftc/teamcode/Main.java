package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
    @TeleOp(name = "FirstOpMode")
    public class Main extends LinearOpMode {

        private DcMotor _0LeftBackMotor;
        private DcMotor _1LeftFrontMotor;
        private DcMotor _2RightBackMotor;
        private DcMotor _3RightFrontMotor;

        /**
         * This function is executed when this Op Mode is selected from the Driver Station.
         */
        @Override
        public void runOpMode() {
            float StrafeR;
            float StrafeL;
            float Down;
            float spin;

            _0LeftBackMotor = hardwareMap.get(DcMotor.class, "0LeftBackMotor");
            _1LeftFrontMotor = hardwareMap.get(DcMotor.class, "1LeftFrontMotor");
            _2RightBackMotor = hardwareMap.get(DcMotor.class, "2RightBackMotor");
            _3RightFrontMotor = hardwareMap.get(DcMotor.class, "3RightFrontMotor");

            waitForStart();
            if (opModeIsActive()) {
                while (opModeIsActive()) {

                    StrafeR = -gamepad1.left_trigger;
                    StrafeL = -gamepad1.right_trigger;
                    Down = gamepad1.left_stick_y;
                    spin = -gamepad1.right_stick_x;

                    _0LeftBackMotor.setPower(StrafeR);
                    _1LeftFrontMotor.setPower(-StrafeR);
                    _2RightBackMotor.setPower(StrafeR);
                    _3RightFrontMotor.setPower(-StrafeR);

                    _0LeftBackMotor.setPower(-StrafeL);
                    _1LeftFrontMotor.setPower(StrafeL);
                    _2RightBackMotor.setPower(-StrafeL);
                    _3RightFrontMotor.setPower(StrafeL);

                    _0LeftBackMotor.setPower(Down);
                    _1LeftFrontMotor.setPower(Down);
                    _2RightBackMotor.setPower(-Down);
                    _3RightFrontMotor.setPower(-Down);

                    _0LeftBackMotor.setPower(spin);
                    _1LeftFrontMotor.setPower(spin);
                    _2RightBackMotor.setPower(spin);
                    _3RightFrontMotor.setPower(spin);

                    telemetry.addData("Up and Down", StrafeL);
                    telemetry.addData("Left and Right", Down);
                    telemetry.update();
                }
            }
        }
    }

}

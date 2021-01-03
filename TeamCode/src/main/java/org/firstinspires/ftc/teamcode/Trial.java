
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Trial)")
public class Trial extends LinearOpMode {

    private DcMotor Left_Rear_Wheel;
    private DcMotor Left_Front_Wheel;
    private DcMotor Right_Rear_Wheel;
    private DcMotor Right_Front_Wheel;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        Left_Rear_Wheel = hardwareMap.get(DcMotor.class, "Left_Rear_Wheel");
        Left_Front_Wheel = hardwareMap.get(DcMotor.class, "Left_Front_Wheel");
        Right_Rear_Wheel = hardwareMap.get(DcMotor.class, "Right_Rear_Wheel");
        Right_Front_Wheel = hardwareMap.get(DcMotor.class, "Right_Front_Wheel");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                if (gamepad1.dpad_up) {
                    Left_Rear_Wheel.setPower(1);
                    Left_Front_Wheel.setPower(1);
                    Right_Rear_Wheel.setPower(1);
                    Right_Front_Wheel.setPower(1);
                }
                if (gamepad1.dpad_down) {
                    Left_Rear_Wheel.setPower(-1);
                    Left_Front_Wheel.setPower(-1);
                    Right_Rear_Wheel.setPower(-1);
                    Right_Front_Wheel.setPower(-1);
                }
                if (gamepad1.dpad_left) {
                    Left_Rear_Wheel.setPower(-1);
                    Left_Front_Wheel.setPower(-1);
                    Right_Rear_Wheel.setPower(1);
                    Right_Front_Wheel.setPower(1);
                }
                if (gamepad1.dpad_right) {
                    Left_Rear_Wheel.setPower(1);
                    Left_Front_Wheel.setPower(1);
                    Right_Rear_Wheel.setPower(-1);
                    Right_Front_Wheel.setPower(-1);
                }
                telemetry.update();
            }
        }
    }
}
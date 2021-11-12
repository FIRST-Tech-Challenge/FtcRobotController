package org.firstinspires.ftc.Team19567;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Carousel Test", group="Iterative Opmode")

public class CarouselTest extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor carouselDC = null;

    public void init() {
        telemetry.addData("Status", "Initialized");

        carouselDC = hardwareMap.get(DcMotor.class, "rightFront");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Waiting for start");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double power = gamepad1.left_stick_y;

        carouselDC.setPower(power);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
    }

}

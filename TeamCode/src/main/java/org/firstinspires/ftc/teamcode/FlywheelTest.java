package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Flywheel Test", group = "Test")
public class FlywheelTest extends LinearOpMode {
    private CRServo flywheelDirect;
    private Flywheel flywheel;
    private ElapsedTime runtime = new ElapsedTime();
    
    private static final double TEST_DURATION = 2.0;
    private static final double[] TEST_POWERS = {0.0, 0.3, 0.6, 1.0};
    
    @Override
    public void runOpMode() {
        flywheelDirect = hardwareMap.get(CRServo.class, "flywheel");
        flywheel = new Flywheel(hardwareMap, "flywheel");
        
        telemetry.addData("Servo Type", flywheelDirect.getClass().getSimpleName());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        // Test different power levels in forward direction
        for (double power : TEST_POWERS) {
            telemetry.addData("Test", String.format("Forward Power: %.1f", power));
            telemetry.update();
            
            flywheelDirect.setPower(power);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < TEST_DURATION) {
                telemetry.addData("Direct Power", power);
                telemetry.update();
                idle();
            }
        }
        
        // Test different power levels in reverse direction
        for (double power : TEST_POWERS) {
            telemetry.addData("Test", String.format("Reverse Power: %.1f", -power));
            telemetry.update();
            
            flywheelDirect.setPower(-power);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < TEST_DURATION) {
                telemetry.addData("Direct Power", -power);
                telemetry.update();
                idle();
            }
        }
        
        // Test using Flywheel class
        telemetry.addData("Test", "Forward Rotation");
        telemetry.update();
        flywheel.start(true);  // Start forward
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < TEST_DURATION) {
            telemetry.addData("Is Running", flywheel.isRunning());
            telemetry.addData("Is Forward", flywheel.isForward());
            telemetry.update();
            idle();
        }
        
        telemetry.addData("Test", "Reverse Rotation");
        telemetry.update();
        flywheel.start(false);  // Start reverse
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < TEST_DURATION) {
            telemetry.addData("Is Running", flywheel.isRunning());
            telemetry.addData("Is Forward", flywheel.isForward());
            telemetry.update();
            idle();
        }
        
        flywheel.stop();
        telemetry.addData("Status", "Test Complete");
        telemetry.update();
        
        while (opModeIsActive()) {
            idle();
        }
    }
} 
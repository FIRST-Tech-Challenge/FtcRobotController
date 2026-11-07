package org.firstinspires.ftc.teamcode.subsystems.drive;





import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Subsystem;

public class MecanumDrive extends Subsystem {

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    boolean slowmode = false;
    private double axial;
    private double yaw;
    private double lateral;

    public MecanumDrive(HardwareMap hardwareMap, Pinpoint pinpoint){

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl"); //0
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl"); //3
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr"); //1
        backRightDrive = hardwareMap.get(DcMotor.class, "br"); //2

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void periodic() {
        double max;



        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.


        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        if (slowmode) {
            frontLeftPower /= 2;
            frontRightPower /= 2;
            backLeftPower /= 2;
            backRightPower /= 2;
        }

        // Send calculated power to wheels
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);


    }
    public void setDriverInput(double axial, double lateral, double yaw){
        this.axial   = axial;
        this.lateral =  lateral;
        this.yaw     =  yaw;

    }

    public void toggleMode(){
        this.slowmode = !slowmode;
    }
    @Override
    public void writeTelemetry(Telemetry telemetry) {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}

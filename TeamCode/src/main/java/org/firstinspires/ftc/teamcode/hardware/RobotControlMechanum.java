package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotGlobalSettings;
import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class RobotControlMechanum {
    enum DrivingAngleMode {AsIs, Nearest90, AtAngle}
    public enum LinearDirection {moveForward, moveBackward, strafeRight, strafeLeft}
    static final String TAG = "RobotControlMechanum";

    RobotHardwareMap robotHardwareMap;
    LinearOpMode opMode;
    public static Orientation lastAngles = new Orientation();
    private static double globalAngle;
    public double correction=0;

    double DrivingAngle = 0;
    double GlobalAngle;
    ElapsedTime runtime     = new ElapsedTime();


    public RobotControlMechanum(RobotHardwareMap robotHardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
    }

    /**
     * Initialization for the mechanum wheels to match up to control
     */
    public void initialize(){
        robotHardwareMap.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardwareMap.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardwareMap.frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardwareMap.frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /***
     * For Teleop run this as passing in these parameters and doing straight power control
     *  using code from https://github.com/brandon-gong/ftc-mecanum/blob/master/MecanumDrive.java
     * @param drive
     * @param strafe
     * @param twist
     */
    public void teleOpMechanum(double drive, double strafe, double twist){
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        robotHardwareMap.frontLeftMotor.setPower(speeds[0]);
        robotHardwareMap.frontRightMotor.setPower(speeds[1]);
        robotHardwareMap.backLeftMotor.setPower(speeds[2]);
        robotHardwareMap.backRightMotor.setPower(speeds[3]);
    }

    /**
     *
     */
    public void strafeHubAlign(){

    }


    /***
     * Code from freight frenzy to do strafe within auton
     * @param dist_Strafe_In
     * @param dir
     * @param power
     * @param timeoutS
     * @param theOpMode
     * @param nearest90
     */
    public void strafe(double dist_Strafe_In, LinearDirection dir, double power, double timeoutS, LinearOpMode theOpMode, boolean nearest90){


        int newStrafeLeftFTarget;
        int newStrafeRightFTarget;
        int newStrafeLeftBTarget;
        int newStrafeRightBTarget;

        robotHardwareMap.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robotHardwareMap.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robotHardwareMap.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robotHardwareMap.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset the encoders
        robotHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardwareMap.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // When we ask to move one tile length, the robot actually moves 33.5"
        // Therefore scale the requested distance
        dist_Strafe_In *= RobotGlobalSettings.OneTileLength_inch / 33.5 ;
        if (dir == LinearDirection.strafeLeft)
        {
            dist_Strafe_In = Math.abs(dist_Strafe_In) * -1;
        }

        //Determine new target postition, and pass to motor controller
        newStrafeLeftFTarget  = (int)(-1*dist_Strafe_In * RobotGlobalSettings.COUNTS_PER_INCH / 0.707);
        newStrafeRightFTarget = (int)(dist_Strafe_In * RobotGlobalSettings.COUNTS_PER_INCH / 0.707);
        newStrafeLeftBTarget  = (int)(dist_Strafe_In * RobotGlobalSettings.COUNTS_PER_INCH / 0.707);
        newStrafeRightBTarget = (int)(-1*dist_Strafe_In * RobotGlobalSettings.COUNTS_PER_INCH / 0.707);

        //Send the target position to the REV module
        robotHardwareMap.backLeftMotor.setTargetPosition(newStrafeLeftFTarget);
        robotHardwareMap.frontLeftMotor.setTargetPosition(newStrafeRightFTarget);
        robotHardwareMap.backRightMotor.setTargetPosition(newStrafeLeftBTarget);
        robotHardwareMap.frontRightMotor.setTargetPosition(newStrafeRightBTarget);

        //Set the motors to run to encoder mode
        robotHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // DrivingAngle is the heading at which we want to drive
        DrivingAngle = nearest90 ? Math.round(getAngle() / 90) * 90 : getAngle();

        //Set the motor speed
        runtime.reset();
        setMotorPowerForLinearMove(power);

        runtime.reset();
        while ((runtime.seconds() < timeoutS)
                && (robotHardwareMap.backLeftMotor.isBusy())
                && !theOpMode.isStopRequested() && theOpMode.opModeIsActive()){

            correction = checkDirection(DrivingAngle);
            if(Math.abs(newStrafeLeftFTarget - robotHardwareMap.frontLeftMotor.getCurrentPosition()) > 150 ) {
                correction *= -1;
            }
            else{
                correction *= 0;
            }
            setMotorPowerForLinearMove(power + correction, power - correction, power, power);

            theOpMode.sleep(1);
           /* theOpMode.telemetry.addData("is_moving drive", is_moving);
            theOpMode.telemetry.addData("LF encoder","position= %d",  robotHardwareMap.RL.getCurrentPosition());
            theOpMode.telemetry.addData("RF encoder","position= %d",  robotHardwareMap.frontLeftMotor.getCurrentPosition());
            theOpMode.telemetry.addData("LR encoder","position= %d",  robotHardwareMap.backRightMotor.getCurrentPosition());
            theOpMode.telemetry.addData("RR encoder","position= %d",  robotHardwareMap.frontRightMotor.getCurrentPosition());
            theOpMode.telemetry.addData("Runtime",runtime.time());
            theOpMode.telemetry.addData("Timeout", timeoutS);
            theOpMode.telemetry.update();
*/
        }

        setMotorPowerForLinearMove(0);

        robotHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double checkDirection(double angle)
    {
        double correction;

        GlobalAngle = getAngle();
        angle = angle - GlobalAngle;

        correction = angle * RobotGlobalSettings.ImuCorrectionFactor;

        return correction;
    }

    protected void setMotorPowerForLinearMove(double LF, double RF, double RR, double LR) {
        robotHardwareMap.frontLeftMotor.setPower(LF);
        robotHardwareMap.frontRightMotor.setPower(RF);
        robotHardwareMap.backRightMotor.setPower(RR);
        robotHardwareMap.backLeftMotor.setPower(LR);
    }

    protected void setMotorPowerForLinearMove(double power) {
        setMotorPowerForLinearMove(power, power, power,power);
    }

    public void resetAngle()
    {
        lastAngles = robotHardwareMap.chImu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robotHardwareMap.chImu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}
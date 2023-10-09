package org.firstinspires.ftc.teamcode.src.v2.Subsystem;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.src.v2.Utility.myDcMotorEx;
import org.firstinspires.ftc.teamcode.src.v2.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.src.v2.maths.mathsOperations;
import org.firstinspires.ftc.teamcode.src.v2.maths.swerveKinematics;

public class SwerveDrive {

    final private IMU imu;
    final private myDcMotorEx mod1m1, mod1m2, mod2m1, mod2m2;
    final private AnalogInput mod1E, mod2E;
    final private Telemetry telemetry;
    final private boolean eff;
    private double module1Adjust = -10, module2Adjust = -10
    private final PIDcontroller mod1PID = new PIDcontroller(0.1, 0.002, 3, 1, 0.5);
    private final PIDcontroller mod2PID = new PIDcontroller(0.1, 0.002, 2, 0.5, 0.5);
    private final swerveKinematics swavemath = new swerveKinematics();

    double mod1reference = 0;
    double mod2reference = 0;
    double heading;
    double imuOffset = 0;


    public SwerveDrive(Telemetry telemetry, HardwareMap hardwareMap, boolean eff) {
        mod1m1 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "LeftTopMotor"));
        mod1m2 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "LeftBottomMotor"));
        mod2m1 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightTopMotor"));
        mod2m2 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightBottomMotor"));
        mod1E = hardwareMap.get(AnalogInput.class, "mod1E");
        mod2E = hardwareMap.get(AnalogInput.class, "mod2E");

        mod1m1.setPowerThresholds(0.05, 0);
        mod1m2.setPowerThresholds(0.05, 0);
        mod2m1.setPowerThresholds(0.05, 0);
        mod2m2.setPowerThresholds(0.05, 0);

        imu = new IMU(hardwareMap);


        this.telemetry = telemetry;
        this.eff = eff;
    }

    public void drive(double x, double y, double rot) {

        //Turn our MA3 absolute encoder signals from volts to degrees
        double mod1P = mod1E.getVoltage() * 74.16;
        double mod2P = mod2E.getVoltage() * 74.16;

        //Update heading of robot
        heading = imu.getHeadingInDegrees();

        //Retrieve the angle and power for each module
        double[] output = swavemath.calculate(y, -x, -rot, heading, true);
        double mod1power = output[0];
        double mod3power = output[1];
        double mod2power = output[2];

        //keep previous module heading if joystick not being used
        if (y != 0 || x != 0 || rot != 0) {
            mod1reference = output[3];
            mod3reference = output[5];
            mod2reference = output[4];
        }

        //set the zero of each module to be forward
        mod2P -= module2Adjust;
        mod1P -= module1Adjust;

        //Anglewrap all the angles so that the module turns both ways
        mod1P = mathsOperations.angleWrap(mod1P);
        mod2P = mathsOperations.angleWrap(mod2P);

        mod1reference = mathsOperations.angleWrap(mod1reference);
        mod2reference = mathsOperations.angleWrap(mod2reference);
        mod3reference = mathsOperations.angleWrap(mod3reference);

        //Make sure that a module never turns more than 90 degrees
        double[] mod1efvalues = mathsOperations.efficientTurn(mod1reference, mod1P, mod1power);

        double[] mod2efvalues = mathsOperations.efficientTurn(mod2reference, mod2P, mod2power);


        if (eff) {
            mod1reference = mod1efvalues[0];
            mod1power = mod1efvalues[1];
            mod2reference = mod2efvalues[0];
            mod2power = mod2efvalues[1];
        }

        //change coax values into diffy values from pid and power
        double[] mod1values = mathsOperations.diffyConvert(mod1PID.pidOut(AngleUnit.normalizeDegrees(mod1reference - mod1P)), -mod1power);
        mod1m1.setPower(mod1values[0]);
        mod1m2.setPower(mod1values[1]);
        double[] mod2values = mathsOperations.diffyConvert(-mod2PID.pidOut(AngleUnit.normalizeDegrees(mod2reference - mod2P)), mod2power);
        mod2m1.setPower(mod2values[0]);
        mod2m2.setPower(mod2values[1]);


        telemetry.addData("mod1reference", mod1reference);
        telemetry.addData("mod2reference", mod2reference);
        telemetry.addData("mod3reference", mod3reference);

        telemetry.addData("mod1P", mod1P);
        telemetry.addData("mod2P", mod2P);
    }

    public void rotateKids(double angle) {
        this.imuOffset = angle;
    }

    public void resetIMU() {
        imu.resetIMU();
    }

    //tune module PIDs
    public void setPIDCoeffs(double Kp, double Kd, double Ki, double Kf, double limit) {
        mod1PID.setPIDgains(Kp, Kd, Ki, Kf, limit);
    }

    //tunable module zeroing
    public void setModuleAdjustments(double module1Adjust, double module2Adjust, double module3Adjust) {
        this.module1Adjust = module1Adjust;
        this.module2Adjust = module2Adjust;
        this.module3Adjust = module3Adjust;
    }

    public double getHeading() {
        return imu.getHeadingInDegrees();
    }
}
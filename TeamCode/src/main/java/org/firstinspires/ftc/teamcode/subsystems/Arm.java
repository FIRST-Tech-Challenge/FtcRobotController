//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.arcrobotics.ftclib.util.InterpLUT;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.PIDCoefficients;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//@Config
//public class Arm extends SubsystemBase {
//
////    public enum ArmState {
////        kDriveModeFromFront,
////        kDriveModeFromFar,
////        kDriveModeFromHighRear,
////        k
////    }
//
//    public static PIDFCoefficients ARM1_PID = new PIDFCoefficients(0.07, 0, 0,0);
//    public static PIDFCoefficients ARM2_PID = new PIDFCoefficients(0.033, 0,0 ,0);
//
//    private final Telemetry t;
//    private String name;
//    private String armPotName;
//    private double potOffset;
//    private double softLowLimit;
//    private double softHighLimit;
//    private final AnalogInput pot;
//    private final InterpLUT angleLookup = new InterpLUT();
//
//    private final DcMotor pivotMotor;
//    private final PIDFController controller;
//    private final PIDFCoefficients pidCoefficients;
//    private double maxOutput;
//    private double minOutput;
//    private double setPoint;
//    private boolean pidEnabled;
//    private double openLoopPower;
//
//
//    public Arm(HardwareMap hardwareMap, Telemetry t) {
//        this.t = t;
//        this.name = name;
//        this.armPotName = armPotName;
//        this.potOffset = potOffset;
//        this.softLowLimit = softLowLimit;
//        this.softHighLimit = softHighLimit;
//        this.pidCoefficients = ARM1_PID;
//        this.maxOutput = maxOutput;
//        this.minOutput = minOutput;
//        //pot = hardwareMap.get(AnalogInput.class, armPotName);
//        angleLookup.add(-1,0);
//        angleLookup.add(0, 0);
//        angleLookup.add(0.108, 15);
//        angleLookup.add(0.26, 30);
//        angleLookup.add(0.388, 45);
//        angleLookup.add(0.51, 60);
//        angleLookup.add(0.62, 75);
//        angleLookup.add(0.735, 90);
//        angleLookup.add(0.84, 105);
//        angleLookup.add(0.949, 120);
//        angleLookup.add(1.065, 135);
//        angleLookup.add(1.194, 150);
//        angleLookup.add(1.313, 165);
//        angleLookup.add(1.483, 180);
//        angleLookup.add(1.666, 195);
//        angleLookup.add(1.881, 210);
//        angleLookup.add(2.144, 225);
//        angleLookup.add(2.472, 240);
//        angleLookup.add(2.864, 255);
//        angleLookup.add(3.33, 270);
//        angleLookup.add(4,270);
//        angleLookup.createLUT();
//
//        pivotMotor = hardwareMap.get(DcMotor.class, "Tilt");
//        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        pivotMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        controller = new PIDFController(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d, 0);
//    }
//
//    public double getAngle() {
//
//        final double potentiometerAngle = angleLookup.get(pot.getVoltage());
//
//        // return potentiometerAngle - potOffset;
//        return (potentiometerAngle - potOffset);
//    }
//
//    public
//
//    public void setPower(double power){
//        openLoopPower = power;
//        setPIDEnabled(false);
//    }
//
//    public void setAngle(double angle){
//        setPoint = angle;
//        setPIDEnabled(true);
//    }
//
//    public void setPIDEnabled(boolean enabled){
//        pidEnabled = enabled;
//    }
//
//    @Override
//    public void periodic(){
//        controller.setPIDF(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d, 0);
//
//        double output;
//        if (pidEnabled) {
//            output = controller.calculate(
//                    getAngle(), setPoint
//            );
//
//            double feedForward = 0;//getAngle().getSin() * pidCoefficients.f;
//            output = output + feedForward;
//
//            t.addData(name + "PIDOutput", output);
//            t.addData(name + "PIDSetPoint", setPoint);
//            t.addData(name + "PIDError", controller.getPositionError());
//
//        } else {
//            controller.reset();
//            output = openLoopPower;
//        }
//
//        if ((output < 0) && (getAngle() <= softLowLimit)) {
//            output = 0;
//        }
//
//        if ((output > 0) && (getAngle() >= softHighLimit)) {
//            output = 0;
//        }
//
//        if (output > maxOutput){
//            output = maxOutput;
//        }
//
//        if (output < minOutput){
//            output = minOutput;
//        }
//
//        pivotMotor.setPower(output);
//
//        //telemetry
//        t.addData(name+"Angle", getAngle());
//        t.addData(name+"Voltage", pot.getVoltage());
//    }
//}
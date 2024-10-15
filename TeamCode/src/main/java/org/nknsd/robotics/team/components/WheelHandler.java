package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;

public class WheelHandler implements NKNComponent {
    private final String flName; // We pass in the names of the wheels during construction so that we can change them easier
    private final String frName;
    private final String blName;
    private final String brName;
    private final String[] invertedNames; // Names in this array are reversed during initialization

    private DcMotor motorFR; private DcMotor motorBR; private DcMotor motorFL; private DcMotor motorBL;


    public WheelHandler(String flName,String frName,String blName,String brName,String[] invertedNames){
        this.flName = flName;
        this.frName = frName;
        this.blName = blName;
        this.brName = brName;

        this.invertedNames = invertedNames;
    }

    // Helper function to check if the given motor name needs to be reversed
    private boolean isInReverseList(String name) {
        for (String s: invertedNames) {
            if (s.equals(name)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        //Get drive motors
        motorFL = hardwareMap.dcMotor.get(flName);
        if (isInReverseList(flName)) { // check if the drive motor is in the list to be reversed
            motorFL.setDirection(DcMotor.Direction.REVERSE);
        }

        motorFR = hardwareMap.dcMotor.get(frName);
        if (isInReverseList(frName)) {
            motorFR.setDirection(DcMotor.Direction.REVERSE);
        }

        motorBL = hardwareMap.dcMotor.get(blName);
        if (isInReverseList(blName)) {
            motorBL.setDirection(DcMotor.Direction.REVERSE);
        }

        motorBR = hardwareMap.dcMotor.get(brName);
        if (isInReverseList(brName)) {
            motorBR.setDirection(DcMotor.Direction.REVERSE);
        }

        return true;
    }

    // Key function of the class
    // Takes in y, x, and turning components of the vector, and converts them to power instructions for omni wheels
    public void vectorToMotion(double y, double x, double turning) {
        motorBR.setPower(y + x - turning);
        motorBL.setPower(y - x + turning);
        motorFR.setPower(y - x - turning);
        motorFL.setPower(y + x + turning);
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {
    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {}

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {}

    @Override
    public String getName() {
        return "WheelHandler";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {}

    public void runMotorFR(double speed) {
        motorFR.setPower(speed);
    }

    public void runMotorFL(double speed) {
        motorFL.setPower(speed);
    }

    public void runMotorBR(double speed) {
        motorBR.setPower(speed);
    }

    public void runMotorBL(double speed) {
        motorBL.setPower(speed);
    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        String msgString = "[" + motorFL.getPower() + ", " + motorFR.getPower() + ", " + motorBL.getPower() + ", " + motorBR.getPower() + "]";
        telemetry.addData("FL, FR, BL, BR", msgString);
    }
}

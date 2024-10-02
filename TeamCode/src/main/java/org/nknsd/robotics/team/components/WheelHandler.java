package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;

public class WheelHandler implements NKNComponent {
    private final String flName;
    private final String frName;
    private final String blName;
    private final String brName;
    private final String[] invertedNames;

    DcMotor motorFR; DcMotor motorBR; DcMotor motorFL; DcMotor motorBL;


    public WheelHandler(String flName,String frName,String blName,String brName,String[] invertedNames){
        this.flName = flName;
        this.frName = frName;
        this.blName = blName;
        this.brName = brName;

        this.invertedNames = invertedNames;
    }

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
        if (isInReverseList(flName)) {
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

    public void vectorToMotion(float y, float x, float turning) {
        motorBR.setPower(y + x - turning);
        motorBL.setPower(y - x + turning);
        motorFR.setPower(y - x - turning);
        motorFL.setPower(y + x + turning);
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {}

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

    @Override
    public void doTelemetry(Telemetry telemetry) {
        String msgString = "[" + motorFL.getPower() + ", " + motorFR.getPower() + ", " + motorBL.getPower() + ", " + motorBR.getPower() + "]";
        telemetry.addData("FL, FR, BL, BR", msgString);
        String[] msgArray = new String[]{"One", "Two"};
        telemetry.addData("test", msgArray);
    }
}

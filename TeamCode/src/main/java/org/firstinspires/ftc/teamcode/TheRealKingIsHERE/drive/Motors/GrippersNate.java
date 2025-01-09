package org.firstinspires.ftc.teamcode.TheRealKingIsHERE.drive.Motors;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GrippersNate {
    public Servo grippers;
    public HardwareMap hardwareMap;
    public void init(HardwareMap hwMap, Telemetry telemetry){
        this.hardwareMap = hwMap;
        grippers = hardwareMap.get(Servo.class, "grippers");
    }



    public double getPosition(){
        return grippers.getPosition();
    }

    public void GripperOpen(){
        //grippers.setPosition();
    }

    public void GripperClosed(){
        //grippers.setPosition();
    }
}

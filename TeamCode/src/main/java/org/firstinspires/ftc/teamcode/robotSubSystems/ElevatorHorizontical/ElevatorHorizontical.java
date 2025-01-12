package org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.PID;

public class ElevatorHorizontical {
    private static DcMotor elevatorMotor;
    private static PID pid = new PID(ElevatorHorizonticalConstants.Kp,ElevatorHorizonticalConstants.Ki,ElevatorHorizonticalConstants.Kd,ElevatorHorizonticalConstants.Kf,ElevatorHorizonticalConstants.iZone,ElevatorHorizonticalConstants.maxSpeed,ElevatorHorizonticalConstants.minSpeed);

    private static float wantedPos;

    public static void init(HardwareMap hardwareMap){
        elevatorMotor = hardwareMap.get(DcMotor.class,"horElevator");
    }

    public static void opreate(ElevatorHorizonticalState elevatorHorizonticalState, final float rightJoyStick, Telemetry telemetry){
                switch(elevatorHorizonticalState){
                    case OPEN:
                        wantedPos = ElevatorHorizonticalConstants.openPos;

                        break;
                    case CLOSE:
                        wantedPos = ElevatorHorizonticalConstants.closedPos;

                        break;
                    case OVERRIDE:
                        wantedPos += ElevatorHorizonticalConstants.overrideFactor * -rightJoyStick;

                        break;
                }
                telemetry.addData("wantedPos",wantedPos);
                telemetry.addData("getPos",getPos());
                telemetry.addData("pid getPos",pid.update(getPos()));

                pid.setWanted(wantedPos);
                elevatorMotor.setPower(pid.update(getPos()));
    }
    public static double getPos() {return elevatorMotor.getCurrentPosition();}
}

package org.firstinspires.ftc.teamcode.PRL.Class;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ActionClass {
    DcMotor Intake;
    public ActionClass(HardwareMap hardwareMap){
        Intake = hardwareMap.get(DcMotor.class,"Intake");
    }

    public void Intake_On(){
        Intake.setPower(1);
    }

    public void Intake_Off(){
        Intake.setPower(0);
    }

    public void Intake_R(){
        Intake.setPower(-1);
    }

    // 슈퍼 스피드 서보 두개로 게코휠 돌려서 인테이크 > 터렛으로 보낼거임 . 단 바로 쏘는걸 방지하기 위해 반대로 돌다가 발사 할때만 정방향 돌리기.
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSubsystem {
    private CRServo intake;

    public IntakeSubsystem(CRServo intake){
        this.intake = intake;
    }
    public void forward(){
        intake.setPower(1);
    }

    public void backward(){
        intake.setPower(-1);
    }

    public void stop(){
        intake.setPower(0);
    }
}

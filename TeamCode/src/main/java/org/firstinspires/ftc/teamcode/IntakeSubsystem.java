package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;

public class IntakeSubsystem {
    private final CRServo intakeFront;
    private final CRServo intakeBack;

    public IntakeSubsystem(CRServo intakeFront, CRServo intakeBack){
        this.intakeFront = intakeFront;
        this.intakeBack = intakeBack;
    }
    public void forward(){
        intakeFront.setPower(1);
        intakeBack.setPower(-1);
    }

    public void backward(){
        intakeFront.setPower(-1);
        intakeBack.setPower(1);
    }
    public void stop(){
        intakeFront.setPower(0);
        intakeBack.setPower(0);
    }

    public void run(double power) {
        intakeFront.setPower(power);
        intakeBack.setPower(-power);
    }
}

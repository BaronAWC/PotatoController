package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmSubsystem extends SubsystemBase {
    private DcMotorEx arm;

    public ArmSubsystem(DcMotorEx arm){
        this.arm = arm;
    }
    public void extend(){
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(-1);
    }

    public void retract(){
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(1);
    }

    public void stop(){
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(0);
    }

    public void runToPosition(int position){
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(position);
    }

    public boolean isFinished(){
        return !arm.isBusy();
    }


}

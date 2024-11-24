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
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setPower(-1);
        arm.setTargetPosition(-10750);
        arm.setPower(-1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void retract(){
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setPower(1);
        arm.setTargetPosition(0);
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void stop(){
        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setPower(0);
    }

    public void runToPosition(int position){
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public boolean isFinished(){
        return !arm.isBusy();
    }


}

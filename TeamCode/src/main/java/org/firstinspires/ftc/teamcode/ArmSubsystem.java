package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmSubsystem extends SubsystemBase {
    private DcMotorEx arm;

    public ArmSubsystem(DcMotorEx arm){
        this.arm = arm;
    }
    public void extend(boolean overrideLimits, boolean slowMode){
        double power = slowMode ? 0.5 : 1;
        if(!overrideLimits) {
            arm.setTargetPosition(-10750);
            arm.setPower(-power);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else{
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(-power);
        }
    }

    public void retract(boolean overrideLimits, boolean slowMode){
        double power = slowMode ? 0.5 : 1;
        if(!overrideLimits) {
            arm.setTargetPosition(0);
            arm.setPower(power);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else{
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(power);
        }

    }

    public void stop(){
        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runToPosition(int position, double power){
        arm.setTargetPosition(position);
        if(position > arm.getCurrentPosition()){
            arm.setPower(power);
        }
        else{
            arm.setPower(-power);
        }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public boolean isFinished(){
        return !arm.isBusy();
    }


}

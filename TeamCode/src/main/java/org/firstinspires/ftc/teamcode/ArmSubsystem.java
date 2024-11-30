package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmSubsystem extends SubsystemBase {
    private final DcMotorEx arm;
    private int startPos;
    private PivotSubsystem pivotSubsystem;

    public ArmSubsystem(DcMotorEx arm){
        this.arm = arm;
        startPos = arm.getCurrentPosition();
    }

    public void setPivotSubsystem(PivotSubsystem pivotSubsystem){
        this.pivotSubsystem = pivotSubsystem;
    }

    public void extend(boolean overrideLimits, boolean slowMode){
        double power = slowMode ? 0.5 : 1;
        if(!overrideLimits) {
            if(pivotSubsystem != null && pivotSubsystem.getCurrentPos() < pivotSubsystem.getStartPos()) {
                arm.setTargetPosition(-8500 + startPos);
            }
            else{
                arm.setTargetPosition(-10750 + startPos);
            }
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
            arm.setTargetPosition(startPos);
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
        arm.setTargetPosition(position + startPos);
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

    public int getPosition(){
        return arm.getCurrentPosition();
    }

    public int getStartPos() { return startPos; }

    public void resetStartPosition() { startPos = arm.getCurrentPosition(); }


}

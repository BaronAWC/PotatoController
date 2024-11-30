package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PivotSubsystem extends SubsystemBase {
    private final DcMotorEx pivot;
    private int startPos;

    public PivotSubsystem(DcMotorEx pivot){
        this.pivot = pivot;
        startPos = pivot.getCurrentPosition();
    }
    public void raise(boolean overrideLimits, boolean slowMode){
        double power = slowMode ? 0.5 : 1;
        if(!overrideLimits){
            pivot.setTargetPosition(startPos + 2950);
            pivot.setPower(power);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot.setPower(power);
        }
    }

    public void lower(boolean overrideLimits, boolean slowMode){
        double power = slowMode ? 0.5 : 1;
        if(!overrideLimits){
            pivot.setTargetPosition(startPos - 3950);
            pivot.setPower(-power);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot.setPower(-power);
        }
    }

    public void stop(){
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setPower(0);
    }

    public void runToPosition(int position, double power){
        pivot.setTargetPosition(position);
        if(position > pivot.getCurrentPosition()){
            pivot.setPower(power);
        }
        else{
            pivot.setPower(-power);
        }
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean isFinished(){
        return !pivot.isBusy();
    }

    public void resetStartPosition(){
        startPos = pivot.getCurrentPosition();
    }

    public int getStartPos(){
        return startPos;
    }

}

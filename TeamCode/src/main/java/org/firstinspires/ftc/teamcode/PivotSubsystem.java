package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PivotSubsystem extends SubsystemBase {
    private DcMotorEx pivot;


    public PivotSubsystem(DcMotorEx pivot){
        this.pivot = pivot;
    }
    public void raise(boolean slowMode){
        double power = slowMode ? 0.5 : 1;
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setPower(power);
    }

    public void lower(boolean slowMode){
        double power = slowMode ? 0.5 : 1;
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setPower(-power);
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


}

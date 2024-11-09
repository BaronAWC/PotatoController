package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PivotSubsystem extends SubsystemBase {
    private DcMotorEx pivot;


    public PivotSubsystem(DcMotorEx pivot){
        this.pivot = pivot;
    }
    public void raise(){
        pivot.setPower(1);
    }

    public void lower(){
        pivot.setPower(-1);
    }

    public void stop(){
        pivot.setPower(0);
    }


}

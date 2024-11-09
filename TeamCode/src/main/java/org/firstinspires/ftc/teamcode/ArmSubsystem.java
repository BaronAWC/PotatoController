package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmSubsystem extends SubsystemBase {
    private DcMotorEx arm;

    public ArmSubsystem(DcMotorEx arm){
        this.arm = arm;
    }
    public void extend(){
        arm.setPower(-1);
    }

    public void retract(){
        arm.setPower(1);
    }

    public void stop(){
        arm.setPower(0);
    }


}

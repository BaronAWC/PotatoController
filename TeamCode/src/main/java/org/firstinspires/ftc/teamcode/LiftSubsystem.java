package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LiftSubsystem extends SubsystemBase {
    private DcMotorEx leftLift;
    private DcMotorEx rightLift;

    public enum Control{
        Left,
        Right,
        Both
    };

    public LiftSubsystem(DcMotorEx leftLift, DcMotorEx rightLift){
        this.leftLift = leftLift;
        this.rightLift = rightLift;
    }

    public void extend(){
        leftLift.setPower(-1);
        rightLift.setPower(-1);
    }

    public void retract(){
        leftLift.setPower(1);
        rightLift.setPower(1);
    }

    public void stop(){
        leftLift.setPower(0);
        rightLift.setPower(0);
    }

}

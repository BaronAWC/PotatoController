package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    public void extend(boolean overrideLimits){
        if(!overrideLimits) {
            leftLift.setTargetPosition(-3350);
            leftLift.setPower(-1);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightLift.setTargetPosition(-3350);
            rightLift.setPower(-1);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else{
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setPower(-1);

            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLift.setPower(-1);
        }

    }

    public void retract(boolean overrideLimits){
        if(!overrideLimits) {
            leftLift.setTargetPosition(0);
            leftLift.setPower(1);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightLift.setTargetPosition(0);
            rightLift.setPower(1);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else{
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setPower(1);

            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLift.setPower(1);
        }
    }

    public void stop(){
        leftLift.setTargetPosition(leftLift.getCurrentPosition());
        leftLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightLift.setTargetPosition(rightLift.getCurrentPosition());
        rightLift.setPower(0);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}

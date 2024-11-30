package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LiftSubsystem extends SubsystemBase {

    public static final int EXTEND_POS = -3700;
    private final DcMotorEx leftLift;
    private final DcMotorEx rightLift;

    private int leftStartPos;
    private int rightStartPos;

    public LiftSubsystem(DcMotorEx leftLift, DcMotorEx rightLift){
        this.leftLift = leftLift;
        this.rightLift = rightLift;

        leftStartPos = leftLift.getCurrentPosition();
        rightStartPos = rightLift.getCurrentPosition();
    }

    public void extend(boolean overrideLimits, boolean leftDown, boolean rightDown){
        if(!overrideLimits) {
            if(leftDown && !rightDown){
                leftLift.setTargetPosition(EXTEND_POS + leftStartPos);
                leftLift.setPower(-1);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightLift.setPower(0);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else if(rightDown && !leftDown){
                rightLift.setTargetPosition(EXTEND_POS + rightStartPos);
                rightLift.setPower(-1);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
           else{
                leftLift.setTargetPosition(EXTEND_POS + leftStartPos);
                leftLift.setPower(-1);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightLift.setTargetPosition(EXTEND_POS + rightStartPos);
                rightLift.setPower(-1);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        else{
            if(leftDown && !rightDown){
                leftLift.setPower(-1);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                rightLift.setPower(0);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else if(rightDown && !leftDown){
                rightLift.setPower(-1);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else{
                leftLift.setPower(-1);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                rightLift.setPower(-1);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

    }

    public void retract(boolean overrideLimits, boolean leftDown, boolean rightDown){
        if(!overrideLimits) {
            if(leftDown && !rightDown){
                leftLift.setTargetPosition(leftStartPos);
                leftLift.setPower(1);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightLift.setPower(0);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else if(rightDown && !leftDown){
                rightLift.setTargetPosition(rightStartPos);
                rightLift.setPower(1);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else{
                leftLift.setTargetPosition(leftStartPos);
                leftLift.setPower(1);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightLift.setTargetPosition(rightStartPos);
                rightLift.setPower(1);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        else{
            if(leftDown && !rightDown){
                leftLift.setPower(1);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                rightLift.setPower(0);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else if(rightDown && !leftDown){
                rightLift.setPower(1);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else{
                leftLift.setPower(1);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                rightLift.setPower(1);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

    }

    public void stop(){
        leftLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift.setPower(0);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetStartPositions(){
        leftStartPos = leftLift.getCurrentPosition();
        rightStartPos = rightLift.getCurrentPosition();
    }

    public int getLeftStartPos(){
        return leftStartPos;
    }

    public int getRightStartPos(){
        return rightStartPos;
    }

}

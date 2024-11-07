package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm {
    public enum ArmDirection{
      Extend,
      Retract,
      Stop
    };
    private DcMotorEx arm;
    public Arm(DcMotorEx arm){
        this.arm = arm;
    }

    public void extend(){
        arm.setPower(1);
    }

    public void retract(){
        arm.setPower(-1);
    }

    public void stop(){
        arm.setPower(0);
    }
}

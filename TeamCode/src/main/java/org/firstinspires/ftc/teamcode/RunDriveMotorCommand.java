package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RunDriveMotorCommand extends CommandBase {
    private final DcMotorEx motor;
    public RunDriveMotorCommand(DcMotorEx motor){
        this.motor = motor;
    }

    @Override
    public void initialize(){
        motor.setPower(1);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

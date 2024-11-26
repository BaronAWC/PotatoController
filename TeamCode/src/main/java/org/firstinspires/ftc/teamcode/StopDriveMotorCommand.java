package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class StopDriveMotorCommand extends CommandBase {
    private final DcMotorEx motor;
    public StopDriveMotorCommand(DcMotorEx motor){
        this.motor = motor;
    }

    @Override
    public void initialize(){
        motor.setPower(0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

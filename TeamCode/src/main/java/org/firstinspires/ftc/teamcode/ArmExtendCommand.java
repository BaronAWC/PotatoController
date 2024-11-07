package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class ArmExtendCommand extends CommandBase {

    private ArmSubsystem armSubsystem;
    public ArmExtendCommand(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize(){
        armSubsystem.extend();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class ArmStopCommand extends CommandBase {

    private ArmSubsystem armSubsystem;
    public ArmStopCommand(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize(){
        armSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

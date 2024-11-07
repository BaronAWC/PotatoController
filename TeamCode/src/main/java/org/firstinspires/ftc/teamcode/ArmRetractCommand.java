package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class ArmRetractCommand extends CommandBase {

    private ArmSubsystem armSubsystem;
    public ArmRetractCommand(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize(){
        armSubsystem.retract();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

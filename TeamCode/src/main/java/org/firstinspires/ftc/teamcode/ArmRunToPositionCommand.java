package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class ArmRunToPositionCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final int position;

    public ArmRunToPositionCommand(ArmSubsystem armSubsystem, int position){
        this.armSubsystem = armSubsystem;
        this.position = position;
    }

    @Override
    public void initialize(){
        armSubsystem.runToPosition(position);
    }

    @Override
    public boolean isFinished(){
        return armSubsystem.isFinished();
    }
}

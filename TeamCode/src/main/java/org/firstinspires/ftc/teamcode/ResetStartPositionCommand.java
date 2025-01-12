package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class ResetStartPositionCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final PivotSubsystem pivotSubsystem;

    public ResetStartPositionCommand(ArmSubsystem armSubsystem, PivotSubsystem pivotSubsystem){
        this.armSubsystem = armSubsystem;
        this.pivotSubsystem = pivotSubsystem;
    }

    @Override
    public void initialize(){
        armSubsystem.resetStartPosition();
        pivotSubsystem.resetStartPosition();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}

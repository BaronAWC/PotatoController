package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class PivotRaiseCommand extends CommandBase {

    private PivotSubsystem pivotSubsystem;
    public PivotRaiseCommand(PivotSubsystem pivotSubsystem){
        this.pivotSubsystem = pivotSubsystem;
    }

    @Override
    public void initialize(){
        pivotSubsystem.raise();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

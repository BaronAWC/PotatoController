package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class PivotRunToPositionCommand extends CommandBase {

    private final PivotSubsystem pivotSubsystem;
    private final int position;

    public PivotRunToPositionCommand(PivotSubsystem pivotSubsystem, int position){
        this.pivotSubsystem = pivotSubsystem;
        this.position = position;
    }

    @Override
    public void initialize(){
        pivotSubsystem.runToPosition(position);
    }

    @Override
    public boolean isFinished(){
        return pivotSubsystem.isFinished();
    }
}

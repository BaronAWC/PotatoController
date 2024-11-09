package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class PivotLowerCommand extends CommandBase {

    private PivotSubsystem pivotSubsystem;
    public PivotLowerCommand(PivotSubsystem pivotSubsystem){
        this.pivotSubsystem = pivotSubsystem;
    }

    @Override
    public void initialize(){
        pivotSubsystem.lower();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

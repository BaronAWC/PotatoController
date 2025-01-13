package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class AutoOffsetCommand extends CommandBase {

    private final PivotSubsystem pivotSubsystem;

    public AutoOffsetCommand(PivotSubsystem pivotSubsystem){
        this.pivotSubsystem = pivotSubsystem;
    }

    @Override
    public void initialize(){
        pivotSubsystem.autoOffset();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

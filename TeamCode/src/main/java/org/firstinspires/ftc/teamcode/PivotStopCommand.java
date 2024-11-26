package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class PivotStopCommand extends CommandBase {

    private final PivotSubsystem pivotSubsystem;
    public PivotStopCommand(PivotSubsystem pivotSubsystem){
        this.pivotSubsystem = pivotSubsystem;
    }

    @Override
    public void initialize(){
        pivotSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

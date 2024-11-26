package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class PivotLowerCommand extends CommandBase {

    private final PivotSubsystem pivotSubsystem;
    private final BooleanSupplier slowMode;
    public PivotLowerCommand(PivotSubsystem pivotSubsystem, BooleanSupplier slowMode){
        this.pivotSubsystem = pivotSubsystem;
        this.slowMode = slowMode;
    }

    @Override
    public void initialize(){
        pivotSubsystem.lower(slowMode.getAsBoolean());
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

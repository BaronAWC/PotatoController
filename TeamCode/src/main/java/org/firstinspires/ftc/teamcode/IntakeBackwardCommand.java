package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class IntakeBackwardCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    public IntakeBackwardCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize(){
        intakeSubsystem.backward();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class IntakeStopCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    public IntakeStopCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize(){
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

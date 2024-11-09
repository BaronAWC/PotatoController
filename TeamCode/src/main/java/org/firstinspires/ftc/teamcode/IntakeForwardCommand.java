package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class IntakeForwardCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;
    public IntakeForwardCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize(){
        intakeSubsystem.forward();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class LiftExtendCommand extends CommandBase {

    private LiftSubsystem liftSubsystem;
    public LiftExtendCommand(LiftSubsystem liftSubsystem) { this.liftSubsystem = liftSubsystem; }

    @Override
    public void initialize(){
        liftSubsystem.extend();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

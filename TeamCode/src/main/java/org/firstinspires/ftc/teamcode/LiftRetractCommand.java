package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class LiftRetractCommand extends CommandBase {

    private LiftSubsystem liftSubsystem;
    public LiftRetractCommand(LiftSubsystem liftSubsystem) { this.liftSubsystem = liftSubsystem; }

    @Override
    public void initialize(){
        liftSubsystem.retract();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

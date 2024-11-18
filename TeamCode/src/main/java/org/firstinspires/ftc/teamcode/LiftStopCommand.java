package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class LiftStopCommand extends CommandBase {

    private LiftSubsystem liftSubsystem;
    public LiftStopCommand(LiftSubsystem liftSubsystem) { this.liftSubsystem = liftSubsystem; }

    @Override
    public void initialize(){
        liftSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

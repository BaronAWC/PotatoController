package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class ClawOpenCommand extends CommandBase {
    private final ClawSubsystem clawSubsystem;
    public ClawOpenCommand(ClawSubsystem clawSubsystem){
        this.clawSubsystem = clawSubsystem;
    }

    @Override
    public void initialize(){
        clawSubsystem.open();
    }

    @Override
    public void end(boolean interrupted){
        clawSubsystem.stop();
    }

}

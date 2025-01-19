package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class ClawCommand extends CommandBase {
    private final ClawSubsystem clawSubsystem;
    private final DoubleSupplier y;
    public ClawCommand(ClawSubsystem clawSubsystem, DoubleSupplier y){
        this.clawSubsystem = clawSubsystem;
        this.y = y;
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute(){
        if(y.getAsDouble() > 0){
            clawSubsystem.open();
        }
        else if(y.getAsDouble() < 0){
            clawSubsystem.close();
        }
        else{
            clawSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted){
        clawSubsystem.stop();
    }

}

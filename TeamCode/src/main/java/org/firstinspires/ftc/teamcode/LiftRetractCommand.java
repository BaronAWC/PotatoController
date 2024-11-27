package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class LiftRetractCommand extends CommandBase {

    private final LiftSubsystem liftSubsystem;
    private final BooleanSupplier overrideLimits;
    public LiftRetractCommand(LiftSubsystem liftSubsystem, BooleanSupplier overrideLimits) {
        this.liftSubsystem = liftSubsystem;
        this.overrideLimits = overrideLimits;
    }

    @Override
    public void execute(){
        liftSubsystem.retract(overrideLimits.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted){
        liftSubsystem.stop();
    }
}

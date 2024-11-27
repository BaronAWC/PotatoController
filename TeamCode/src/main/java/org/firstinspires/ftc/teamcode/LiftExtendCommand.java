package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class LiftExtendCommand extends CommandBase {

    private final LiftSubsystem liftSubsystem;
    private final BooleanSupplier overrideLimits;
    public LiftExtendCommand(LiftSubsystem liftSubsystem, BooleanSupplier overrideLimits) {
        this.liftSubsystem = liftSubsystem;
        this.overrideLimits = overrideLimits;
    }

    @Override
    public void execute(){
        liftSubsystem.extend(overrideLimits.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted){
        liftSubsystem.stop();
    }
}

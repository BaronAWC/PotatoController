package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class LiftRetractCommand extends CommandBase {

    private LiftSubsystem liftSubsystem;
    private final BooleanSupplier overrideLimits;
    public LiftRetractCommand(LiftSubsystem liftSubsystem, BooleanSupplier overrideLimits) {
        this.liftSubsystem = liftSubsystem;
        this.overrideLimits = overrideLimits;
    }

    @Override
    public void initialize(){
        liftSubsystem.retract(overrideLimits.getAsBoolean());
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

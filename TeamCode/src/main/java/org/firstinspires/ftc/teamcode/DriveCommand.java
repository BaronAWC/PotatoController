package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    // see DefaultDrive in examples
    private final DriveSubsystem driveSubsystem;
    private final double x, y, rx;
    private final boolean isSlow;

    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rx, BooleanSupplier isSlow){
        this.driveSubsystem = driveSubsystem;
        this.x = x.getAsDouble();
        this.y = y.getAsDouble();
        this.rx = rx.getAsDouble();
        this.isSlow = isSlow.getAsBoolean();
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute(){
        driveSubsystem.drive(x, y, rx, isSlow);
    }
}

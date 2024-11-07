package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class DriveCommand extends CommandBase {
    // see DefaultDrive in examples
    private final DriveSubsystem driveSubsystem;
    private final double x, y, rx;
    private final boolean isSlow;

    public DriveCommand(DriveSubsystem driveSubsystem, double x, double y, double rx, boolean isSlow){
        this.driveSubsystem = driveSubsystem;
        this.x = x;
        this.y = y;
        this.rx = rx;
        this.isSlow = isSlow;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute(){
        driveSubsystem.drive(x, y, rx, isSlow);
    }
}

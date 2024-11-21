package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class DriveStopCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;

    public DriveStopCommand(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize(){
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

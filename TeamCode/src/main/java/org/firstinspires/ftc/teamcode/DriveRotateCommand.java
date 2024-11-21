package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class DriveRotateCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;

    public DriveRotateCommand(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

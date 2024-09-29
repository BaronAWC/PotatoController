package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

public class DriveCommand extends CommandBase {

    private MecanumDrive m_drive;
    private OdometrySubsystem odometry;

    public DriveCommand(MecanumDrive m_drive, OdometrySubsystem odometry){
        this.m_drive = m_drive;
        this.odometry = odometry;
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
        return false;
    }
}

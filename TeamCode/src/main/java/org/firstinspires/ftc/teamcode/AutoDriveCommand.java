package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoDriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final double distance, driveAngle, rotateAngle, speed;
    private final Telemetry telemetry;
    private boolean finishedDriving = false, finishedRotating = false;

    public AutoDriveCommand(DriveSubsystem driveSubsystem, double distance, double driveAngle, double rotateAngle, double speed, Telemetry telemetry){
        this.driveSubsystem = driveSubsystem;
        this.distance = distance;
        this.driveAngle = driveAngle;
        this.rotateAngle = rotateAngle;
        this.speed = speed;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize(){
        driveSubsystem.resetEncoders();
    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return finishedDriving && finishedRotating;
    }


}

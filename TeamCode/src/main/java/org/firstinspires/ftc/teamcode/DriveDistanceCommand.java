package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class DriveDistanceCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final double distance, angle, speed;
    public DriveDistanceCommand(DriveSubsystem driveSubsystem, double distance, double angle, double speed){
        this.driveSubsystem = driveSubsystem;
        this.distance = distance;
        this.angle = Math.toRadians(angle); // angle to drive at
        this.speed = speed;
    }

    @Override
    public void initialize(){
        driveSubsystem.resetEncoders();
        driveSubsystem.setDrive(angle, speed, false);
    }

//    @Override
//    public void execute(){
//        driveSubsystem.autoDrive();
//    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.setDrive(angle, speed, true);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(driveSubsystem.getAverageEncoderDistance()) >= distance; // drive until average of the encoders reaches the distance
    }

}

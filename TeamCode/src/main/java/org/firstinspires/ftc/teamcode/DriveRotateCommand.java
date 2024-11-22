package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class DriveRotateCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final double angle, speed;
    private double startAngle;

    public DriveRotateCommand(DriveSubsystem driveSubsystem, double angle, double speed){
        this.driveSubsystem = driveSubsystem;
        this.angle = angle;
        this.speed = speed;
    }

    @Override
    public void initialize(){
        startAngle = driveSubsystem.getAngle();
        driveSubsystem.setRotation(angle, speed, true);
    }

//    @Override
//    public void execute(){
//        driveSubsystem.autoDrive();
//    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.setRotation(angle, speed, true);
    }

    @Override
    public boolean isFinished(){
        double currentAngle = driveSubsystem.getAngle();
        return ((startAngle + angle - currentAngle) <= 3) || (angle > 0 && currentAngle > (startAngle + angle)) || (angle < 0 && currentAngle < (startAngle - angle));
        // finished if it is off by 3 degrees or if it overshot
    }
}

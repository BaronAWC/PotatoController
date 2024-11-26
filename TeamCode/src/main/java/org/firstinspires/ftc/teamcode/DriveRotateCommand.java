package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveRotateCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final double angle, speed;
    private double startAngle;
    private final Telemetry telemetry;

    public DriveRotateCommand(DriveSubsystem driveSubsystem, double angle, double speed, Telemetry telemetry){
        this.driveSubsystem = driveSubsystem;
        this.angle = angle;
        this.speed = speed;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize(){
        startAngle = driveSubsystem.getAngle();
        driveSubsystem.setRotation(angle, speed, true);
        telemetry.addLine("started drive rotate command " + angle + " " + speed);
        telemetry.update();
    }

//    @Override
//    public void execute(){
//        driveSubsystem.autoDrive();
//    }

    @Override
    public void end(boolean interrupted){
        telemetry.addLine("finished drive rotate command " + angle + " " + speed);
        telemetry.update();
        driveSubsystem.setRotation(angle, speed, true);
    }

    @Override
    public boolean isFinished(){
        double currentAngle = driveSubsystem.getAngle();
        return ((startAngle + angle - currentAngle) <= 3) || (angle > 0 && currentAngle > (startAngle + angle)) || (angle < 0 && currentAngle < (startAngle - angle));
        // finished if it is off by 3 degrees or if it overshot
    }
}

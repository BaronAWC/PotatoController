package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class DriveRotateCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final double angle, speed;
    private final Telemetry telemetry;

    public DriveRotateCommand(DriveSubsystem driveSubsystem, double angle, double speed, Telemetry telemetry){
        this.driveSubsystem = driveSubsystem;
        this.angle = angle;
        this.speed = speed;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize(){
        driveSubsystem.resetEncoders();
        driveSubsystem.setRotation(driveSubsystem.getAngle() - angle, speed, false);
        telemetry.addLine("started drive rotate command " + angle + " " + speed);
        telemetry.update();
    }

    @Override
    public void execute(){
//        telemetry.addData("current angle", driveSubsystem.getAngle());
//        telemetry.addData("difference", (angle - driveSubsystem.getAngle()));
//        telemetry.addData("Front left change", driveSubsystem.getFLChange());
//        telemetry.addData("Back right change", driveSubsystem.getBRChange());
//        telemetry.addData("Front right change", driveSubsystem.getFRChange());
//        telemetry.addData("Back left change", driveSubsystem.getBLChange());
//        telemetry.update();
    }

    @Override
    public void end(boolean interrupted){
        telemetry.addLine("finished drive rotate command " + angle + " " + speed);
        telemetry.update();
        driveSubsystem.setRotation(driveSubsystem.getAngle() - angle, speed, true);
        //driveSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        double currentAngle = driveSubsystem.getAngle();
        return (Math.abs(angle - currentAngle) <= 1.5) || (angle > 0 && currentAngle > (angle)) ||
                (angle < 0 && currentAngle < (angle));
    }
}

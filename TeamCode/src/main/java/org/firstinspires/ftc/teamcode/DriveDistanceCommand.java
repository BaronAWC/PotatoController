package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveDistanceCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final double distance, angle, speed;
    private Telemetry telemetry;
    public DriveDistanceCommand(DriveSubsystem driveSubsystem, double distance, double angle, double speed, Telemetry telemetry){
        this.driveSubsystem = driveSubsystem;
        this.distance = distance;
        this.angle = Math.toRadians(angle); // angle to drive at
        this.speed = speed;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize(){
        driveSubsystem.resetEncoders();
        driveSubsystem.setDrive(angle, speed, false);
        telemetry.addLine("started drive distance command " + distance + " " + angle + " " + speed);
        telemetry.update();
    }

//    @Override
//    public void execute(){
//        driveSubsystem.autoDrive();
//    }
    @Override
    public void execute(){
        telemetry.addData("average encoder distance", driveSubsystem.getAverageEncoderDistance());
        for(Pair<String, String> pair : driveSubsystem.getInfo()){
            telemetry.addData(pair.first, pair.second);
        }
        telemetry.addLine("#############");
        telemetry.update();
    }

    @Override
    public void end(boolean interrupted){
        telemetry.addLine("finished drive distance command " + distance + " " + angle + " " + speed);
        telemetry.update();
        driveSubsystem.setDrive(angle, speed, true);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(driveSubsystem.getAverageEncoderDistance()) >= distance; // drive until average of the encoders reaches the distance
    }

}

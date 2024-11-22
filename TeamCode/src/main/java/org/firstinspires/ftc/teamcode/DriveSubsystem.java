package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem extends SubsystemBase {
    // see DriveSubsystem in examples
    private final MecanumDrive drive;
    private final Encoder FLEncoder, FREncoder, BLEncoder, BREncoder;

    private BHI260IMU imu;

    public DriveSubsystem(DcMotorEx FrontL, DcMotorEx FrontR, DcMotorEx BackL, DcMotorEx BackR, BHI260IMU imu){
        drive = new MecanumDrive(FrontL, FrontR, BackL, BackR, imu);
        this.imu = imu;
        FLEncoder = ((Motor)FrontL).encoder;
        FREncoder = ((Motor)FrontR).encoder;
        BLEncoder = ((Motor)BackL).encoder;
        BREncoder = ((Motor)BackR).encoder;
    }

    public void drive(double x, double y, double rx, boolean isSlow){
        drive.driveFieldCentric(x, y, rx, isSlow);
    }

    public void resetEncoders(){
        FLEncoder.reset();
        FREncoder.reset();
        BLEncoder.reset();
        BREncoder.reset();
    }

    public double getAverageEncoderDistance(){ // probably need to change this based on what the encoder is reporting
        return (FLEncoder.getDistance() + FREncoder.getDistance() + BLEncoder.getDistance() +
                BREncoder.getDistance()) / 4.0;
    }

    public double getAngle(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void stop(){
        drive.stop();
    }

//    public void autoDrive(){
//        drive.autoDrive();
//    }

    public void setDrive(double angle, double speed, boolean end){
        drive.setDrive(angle, speed, end);
    }

    public void setRotation(double angle, double speed, boolean end){
        drive.setRotation(angle, speed, end);
    }
}

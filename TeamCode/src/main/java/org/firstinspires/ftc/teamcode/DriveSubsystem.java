package org.firstinspires.ftc.teamcode;

import android.util.Pair;

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
    private final DcMotorEx FrontL, FrontR, BackL, BackR;
    private double FLStartPos, FRStartPos, BLStartPos, BRStartPos;
    //private final Encoder FLEncoder, FREncoder, BLEncoder, BREncoder;

    private final BHI260IMU imu;

    public DriveSubsystem(DcMotorEx FrontL, DcMotorEx FrontR, DcMotorEx BackL, DcMotorEx BackR, BHI260IMU imu){
        drive = new MecanumDrive(FrontL, FrontR, BackL, BackR, imu);
        this.imu = imu;
        this.FrontL = FrontL;
        this.FrontR = FrontR;
        this.BackL = BackL;
        this.BackR = BackR;
    }

    public void drive(double x, double y, double rx, boolean isSlow){
        drive.driveFieldCentric(x, y, rx, isSlow);
    }

    public void resetEncoders(){
        FLStartPos = FrontL.getCurrentPosition();
        FRStartPos = FrontR.getCurrentPosition();
        BLStartPos = BackL.getCurrentPosition();
        BRStartPos = BackR.getCurrentPosition();
    }

    public double getAverageEncoderDistance(){ // probably need to change this based on what the encoder is reporting
        return ((FrontL.getCurrentPosition() - FLStartPos) + (FrontR.getCurrentPosition() - FRStartPos) + (BackL.getCurrentPosition() - BLStartPos) +
                (BackR.getCurrentPosition() - BRStartPos)) / 4.0;
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

    public Pair<String, String>[] getInfo(){
        //return drive.getInfo();
        return new Pair[]{
                new Pair<String, String>("Front Left Encoder Distance", (FrontL.getCurrentPosition() - FLStartPos) + ""),
                new Pair<String, String>("Front Right Encoder Distance", (FrontR.getCurrentPosition() - FRStartPos) + ""),
                new Pair<String, String>("Back Left Encoder Distance", (BackL.getCurrentPosition() - BLStartPos) + ""),
                new Pair<String, String>("Back Right Encoder Distance", (BackR.getCurrentPosition() - BRStartPos) + "")
        };

    }
}

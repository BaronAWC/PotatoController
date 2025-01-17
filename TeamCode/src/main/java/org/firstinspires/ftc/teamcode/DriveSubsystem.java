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
    private int FLStartPos, FRStartPos, BLStartPos, BRStartPos;
    private double startAngle;

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

    public void setStartAngle(){
        startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public int getFLChange(){
        return FrontL.getCurrentPosition() - FLStartPos;
    }

    public int getFRChange(){
        return FrontR.getCurrentPosition() - FRStartPos;
    }

    public int getBLChange(){
        return BackL.getCurrentPosition() - BLStartPos;
    }

    public int getBRChange(){
        return BackR.getCurrentPosition() - BRStartPos;
    }

    public double getAngle(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getStartAngle() { return startAngle; }

    public void stop(){
        drive.stop();
    }

    public void autoDrive(double startAngle, double currentAngle, double driveAngle, double rotateAngle, boolean rotate, double driveSpeed, double rotateSpeed){
        drive.autoDrive(startAngle, currentAngle, driveAngle, rotateAngle, rotate, driveSpeed, rotateSpeed);
    }
}

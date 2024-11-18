package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem extends SubsystemBase {
    // see DriveSubsystem in examples
    private final MecanumDrive drive;
    private final Telemetry telemetry;
    private BHI260IMU imu;

    public DriveSubsystem(DcMotorEx FrontL, DcMotorEx FrontR, DcMotorEx BackL, DcMotorEx BackR, BHI260IMU imu, Telemetry telemetry){
        drive = new MecanumDrive(FrontL, FrontR, BackL, BackR, imu, telemetry);
        this.telemetry = telemetry;
        this.imu = imu;
    }

    public void drive(double x, double y, double rx, boolean isSlow){
        drive.driveFieldCentric(x, y, rx, isSlow);
        //telemetry.addLine(x + " " + y + " " + rx + " " + isSlow);
        //telemetry.addLine(imu.getRobotYawPitchRollAngles().toString());
        telemetry.update();
    }
}

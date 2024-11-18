package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    // see MecanumDrive in examples
    final DcMotorEx FrontL;
    final DcMotorEx FrontR;
    final DcMotorEx BackL;
    final DcMotorEx BackR;
    final BHI260IMU imu;
    static final int ADJ_FACTOR = 50;
    static final double MAX_DRIVE_PWR = 0.80;
    static final double X_AXIS_ADJ = 1.15; // x axis is a bit slower than y axis on strafer wheels
    static final double SLOW_MODE_POWER = 0.4;
    final double powerFactor = 1;
    private Telemetry telemetry;

    public MecanumDrive(DcMotorEx FrontL, DcMotorEx FrontR, DcMotorEx BackL, DcMotorEx BackR, BHI260IMU imu, Telemetry telemetry){
        this.FrontL = FrontL;
        this.FrontR = FrontR;
        this.BackL = BackL;
        this.BackR = BackR;
        this.imu = imu;
        this.telemetry = telemetry;
    }

    public void driveFieldCentric(double x, double y, double rx, boolean isSlow){
        Pair<Double, Double> xy = joystickToBotPerspective(x, y);
        x = xy.first;
        y = xy.second;

        // lower joystick sensitivity
        y = adjPwr(y, isSlow);
        x = adjPwr(x, isSlow) * X_AXIS_ADJ;
        rx = adjPwr(rx * MAX_DRIVE_PWR, isSlow);

        double xPower = x * powerFactor;
        double yPower = y * powerFactor;

        double fLPwr = yPower + xPower + rx;
        double bLPwr = yPower - xPower + rx;
        double fRPwr = yPower + xPower - rx;
        double bRPwr = yPower - xPower - rx;

        // Put powers in the range of -1 to 1 only if they aren't already (not
        // checking would cause us to always drive at full speed)

        if (Math.abs(fLPwr) > MAX_DRIVE_PWR || Math.abs(bLPwr) > MAX_DRIVE_PWR ||
                Math.abs(fRPwr) > MAX_DRIVE_PWR || Math.abs(bRPwr) > MAX_DRIVE_PWR) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(fLPwr), Math.abs(bLPwr));
            max = Math.max(Math.abs(fRPwr), max);
            max = Math.max(Math.abs(bRPwr), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs), scale everything to the max
            fLPwr /= max;
            bRPwr /= max;
            fRPwr /= max;
            bRPwr /= max;
        }

        driveWithMotorPowers(fLPwr, fRPwr, bLPwr, bRPwr);
    }

    private double adjPwr(Double n, boolean isSlow) {
        // when fine-tuning, just do linear scale so the max power is 25%
        if(isSlow) {
            return n * SLOW_MODE_POWER;
        } else {
            int nSign = n.compareTo((double) 0);
            return Math.pow(Math.abs(n), 1.5) * nSign;
        }
    }

    private Pair<Double, Double> joystickToBotPerspective(double jsX, double jsY) {
        double jsAngle = Math.atan2(jsY,jsX);

        // since the Java math library uses radians, angles from the IMU are also radians
        double jsMagnitude = Math.sqrt(jsX * jsX + jsY * jsY);
        double botAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // "Math.PI / 2" is for the offset between the driver orientation and the robot
        // orientation at the start of tele-op
        double targetAngle = jsAngle;// - botAngle - (Math.PI / 2);
        double botX = jsMagnitude * Math.cos(targetAngle);
        double botY = jsMagnitude * Math.sin(targetAngle);
        //telemetry.addLine("angles: " + (jsAngle * 180 / Math.PI) + " " + (botAngle * 180 / Math.PI));
        //telemetry.addLine("target: " + (targetAngle * 180 / Math.PI) + " " + botX + " " + botY);
        return new Pair<>(botX, botY);
    }

    public void driveWithMotorPowers(double FrontLSpeed, double FrontRSpeed, double BackLSpeed, double BackRSpeed){
        // add any necessary multipliers here
        FrontL.setPower(FrontLSpeed * MAX_DRIVE_PWR);
        FrontR.setPower(FrontRSpeed * MAX_DRIVE_PWR);
        BackL.setPower(BackLSpeed * MAX_DRIVE_PWR);
        BackR.setPower(BackRSpeed * MAX_DRIVE_PWR);
    }
}

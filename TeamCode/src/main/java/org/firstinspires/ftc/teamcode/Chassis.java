package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Chassis {
    final DcMotor FrontL;
    final DcMotor FrontR;
    final DcMotor BackL;
    final DcMotor BackR;

    final Telemetry telemetry;
    final BNO055IMU imu;

    static final int ADJ_FACTOR = 50;
    static final double MAX_DRIVE_PWR = 0.80;
    static final double X_AXIS_ADJ = 1.15; // x axis is a bit slower than y axis on strafer wheels
    static final double SLOW_MODE_POWER = 0.4;
    // TODO: aaron's code has some stuff to calculate a power factor based on battery
    final double powerFactor = 1;

    public Chassis(HardwareMap hardwareMap, Telemetry telemetry) {
        FrontL = hardwareMap.get(DcMotor.class, "FrontL");
        FrontR = hardwareMap.get(DcMotor.class, "FrontR");
        BackL = hardwareMap.get(DcMotor.class, "BackL");
        BackR = hardwareMap.get(DcMotor.class, "BackR");

        this.telemetry = telemetry;
        imu = getIMU(hardwareMap);
    }

    void gamepadDrive(double y, double x, double rx, boolean isSlow) {
        Pair<Double, Double> xy = joystickToBotPerspective(x, y);
        x = xy.first;
        y = xy.second;

        // lower joystick sensitivity
        y = adjPwr(y, isSlow);
        x = adjPwr(x, isSlow) * X_AXIS_ADJ;
        rx = adjPwr(rx * MAX_DRIVE_PWR, isSlow);

        setDrivePwr(y, x, rx);
    }

    private void setDrivePwr(double yPower, double xPower, double rx) {
        // if the bot is too close to the wall, stop the robot from hitting it
        xPower *= powerFactor;
        yPower *= powerFactor;

        double fLPwr = yPower + xPower + rx;
        double bLPwr = yPower - xPower + rx;
        double fRPwr = yPower - xPower - rx;
        double bRPwr = yPower + xPower - rx;

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

        FrontL.setPower(fLPwr * MAX_DRIVE_PWR);
        BackL.setPower(bLPwr * MAX_DRIVE_PWR);
        FrontR.setPower(fRPwr * MAX_DRIVE_PWR);
        BackR.setPower(bRPwr * MAX_DRIVE_PWR);
    }


    double adjPwr(Double n, boolean isSlow) {
        // when fine-tuning, just do linear scale so the max power is 25%
        if(isSlow) {
            return n * SLOW_MODE_POWER;
        } else {
            int nSign = n.compareTo((double) 0);
            return Math.pow(Math.abs(n), 1.5) * nSign;
        }
    }

    /**
     * Translate joystick (js) x and y to robot (bot) x and y.
     * @param jsX
     * @param jsY
     * @return pair of axes which x is the first value and y is the second.
     */
    Pair<Double, Double> joystickToBotPerspective(double jsX, double jsY) {
        double jsAngle = Math.atan2(jsY,jsX);

        // since the Java math library uses radians, angles from the IMU are also radians
        double jsMagnitude = Math.sqrt(jsX * jsX + jsY * jsY);
        double botAngle = getAngleRadians();

        // "Math.PI / 2" is for the offset between the driver orientation and the robot
        // orientation at the start of tele-op
        double targetAngle = jsAngle - botAngle - (Math.PI / 2);
        double botX = jsMagnitude * Math.cos(targetAngle);
        double botY = jsMagnitude * Math.sin(targetAngle);
        return new Pair<>(botX, botY);
    }

    double getAngleRadians() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle;
    }

    private BNO055IMU getIMU(HardwareMap hardwareMap) {
        // from sample code
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        return imu;
    }
}

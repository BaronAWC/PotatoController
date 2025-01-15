package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoDriveCommand extends CommandBase {

    public static final double TICKS_PER_REV = 537.7;
    public static final double WHEEL_CIRCUMFERENCE = 10.4 * Math.PI; // in cm
    public static final double MULTIPLIER = 1.414;
    private final DriveSubsystem driveSubsystem;
    private final double distance, driveAngle, rotateAngle, driveSpeed, rotateSpeed;

    private double FL_and_BR_TargetChange, FR_and_BL_TargetChange, FL_and_BR_Theta, FR_and_BL_Theta;
    private double FL_previous = 0, FL_change = 0, BR_previous = 0, BR_change = 0;
    private double FR_previous = 0, FR_change = 0, BL_previous = 0, BL_change = 0;
    private final Telemetry telemetry;
    private boolean finishedDriving = false, finishedRotating = false;

    public AutoDriveCommand(DriveSubsystem driveSubsystem, double distance, double driveAngle, double rotateAngle, double driveSpeed, double rotateSpeed, Telemetry telemetry){
        this.driveSubsystem = driveSubsystem;
        this.distance = distance;
        this.driveAngle = driveAngle;
        this.rotateAngle = rotateAngle;
        this.driveSpeed = driveSpeed;
        this.rotateSpeed = rotateSpeed;
        this.telemetry = telemetry;

        if(distance == 0 || driveSpeed == 0) finishedDriving = true;
        if(rotateSpeed == 0) finishedRotating = true;
    }

    @Override
    public void initialize(){

        driveSubsystem.resetEncoders();
        driveSubsystem.setStartAngle();

        FL_and_BR_Theta = (driveAngle <= 45 && driveAngle >= -90) ? Math.abs(driveAngle + 45) : (135 - driveAngle);
        FL_and_BR_TargetChange = distance * Math.cos(Math.toRadians(FL_and_BR_Theta)) * TICKS_PER_REV / WHEEL_CIRCUMFERENCE * MULTIPLIER;

        FR_and_BL_Theta = (driveAngle <= 90 && driveAngle >= -45) ? Math.abs(driveAngle - 45) : (135 - Math.abs(driveAngle));
        FR_and_BL_TargetChange = distance * Math.cos(Math.toRadians(FR_and_BL_Theta)) * TICKS_PER_REV / WHEEL_CIRCUMFERENCE * MULTIPLIER;

    }

    @Override
    public void execute(){ // TODO test this command
        double FL_current = driveSubsystem.getFLChange();
        FL_change += Math.abs(FL_current - FL_previous);
        FL_previous = FL_current;

        double BR_current = driveSubsystem.getBRChange();
        BR_change += Math.abs(BR_current - BR_previous);
        BR_previous = BR_current;

        telemetry.addData("FL and BR Target Change", FL_and_BR_TargetChange);
        telemetry.addData("FL change", driveSubsystem.getFLChange());
        telemetry.addData("BR change", driveSubsystem.getBRChange());
        telemetry.addData("Fl and BR avg", (driveSubsystem.getFLChange() + driveSubsystem.getBRChange()) / 2);
        telemetry.addData("FL accumulation", FL_change);
        telemetry.addData("BR accumulation", BR_change);
        telemetry.addData("FL and BR accumulation avg", (FL_change + BR_change) / 2);
        telemetry.addLine("#####################");
        telemetry.addData("FR and BL Target Change", FR_and_BL_TargetChange);
        telemetry.addData("FR change", driveSubsystem.getFRChange());
        telemetry.addData("BL change", driveSubsystem.getBLChange());
        telemetry.addData("FR and BL avg", (driveSubsystem.getFRChange() + driveSubsystem.getBLChange()) / 2);
        telemetry.addLine("#####################");
        telemetry.addData("Current angle", driveSubsystem.getAngle());
        telemetry.addData("Start angle", driveSubsystem.getStartAngle());
        telemetry.addData("Target angle", rotateAngle);
        telemetry.update();


        if(!finishedRotating) {
            double currentAngle = driveSubsystem.getAngle();
            finishedRotating =  (Math.abs(rotateAngle - currentAngle) <= 1) || (driveSubsystem.getStartAngle() < rotateAngle && currentAngle > (rotateAngle)) ||
                    (driveSubsystem.getStartAngle() > rotateAngle && currentAngle < (rotateAngle));
        }
        if(!finishedDriving){
            // check if driving is finished
            finishedDriving = ((Math.abs(driveSubsystem.getFLChange()) >= FL_and_BR_TargetChange) && (Math.abs(driveSubsystem.getFRChange()) >= FR_and_BL_TargetChange) &&
                    (Math.abs(driveSubsystem.getBRChange()) >= FL_and_BR_TargetChange) && (Math.abs(driveSubsystem.getBLChange()) >= FR_and_BL_TargetChange)) ||
                    (Math.abs((driveSubsystem.getFLChange() + driveSubsystem.getBRChange()) / 2) >=  FL_and_BR_TargetChange &&
                            Math.abs((driveSubsystem.getFRChange() + driveSubsystem.getBLChange()) / 2) >= FR_and_BL_TargetChange);
        }

        driveSubsystem.autoDrive(driveSubsystem.getStartAngle(), driveSubsystem.getAngle(), driveAngle, rotateAngle, !finishedRotating, (finishedDriving) ? 0 : driveSpeed, rotateSpeed);
    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return finishedDriving && finishedRotating;
    }


}

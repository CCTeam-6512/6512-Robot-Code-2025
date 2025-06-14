package frc.robot.commands;

// import java.security.cert.TrustAnchor;
// import edu.wpi.first.math.proto.System;
import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, throttle;
    private final Supplier<Boolean> fieldOrientedFunction, slowTurn;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private double throttleadjusted;

    public SwerveJoystickCmd(
        SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpdFunction, 
        Supplier<Double> ySpdFunction, 
        Supplier<Double> turningSpdFunction,
        Supplier<Boolean> fieldOrientedFunction,
        Supplier<Double> throttle,
        Supplier<Boolean> slowTurn
        ) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.throttle = throttle;
        this.slowTurn = slowTurn;
        addRequirements(swerveSubsystem);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Front Left Spd", swerveSubsystem.frontLeft.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Front Right Spd", swerveSubsystem.frontRight.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Back Left Spd", swerveSubsystem.backLeft.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Back Right Spd", swerveSubsystem.backRight.getState().speedMetersPerSecond);
        
        
        // 1. Get real-time joystick inputs
        throttleadjusted = throttle.get() * -0.25 + 0.75;
        // throttleadjusted = throttle.get();
        // double xSpeed = applySineInOut(xSpdFunction.get() * throttleadjusted);
        double xSpeed = xSpdFunction.get() * throttleadjusted;
        // double ySpeed = applySineInOut(ySpdFunction.get()* throttleadjusted);
        double ySpeed = ySpdFunction.get() * throttleadjusted;
        // double turningSpeed = applySineInOut(turningSpdFunction.get());
        double turningSpeed = turningSpdFunction.get();


        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDriveDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDriveDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kRotDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        // Slow turning speed on true of slowTurn button boolean supplier
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        if (slowTurn.get()==true){
            turningSpeed=turningSpeed/2;
        }

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
//TODO - for testing, add '!' to make robot oriented by default
        if (!fieldOrientedFunction.get()) {
            // Relative to field
                SmartDashboard.putString("mode","Field");
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
                SmartDashboard.putString("mode","Robot");
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double applySineInOut(double input) {
        double sign = Math.signum(input);
        input = Math.abs(input);
        return sign * 0.5 * (1 - Math.cos(Math.PI * input));
    }
}

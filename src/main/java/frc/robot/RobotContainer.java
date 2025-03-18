package frc.robot;

import java.util.List;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.commands.SwerveJoystickCmd;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final Shooter shooter = new Shooter();
    private final Arm arm = new Arm();
    private final Climber climber = new Climber();

    private final static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final static Joystick xbox = new Joystick(ScoringConstants.kScoringControllerPort);

    public RobotContainer() {

        configureButtonBindings();

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverThrottleAxis),
                () -> driverJoytick.getRawButton(OIConstants.kDriverSlowTurnButtonIdx)
                ));
}

// SendableChooser<Command> chooser = new SendableChooser<>();

final Command ShootPiece = new ParallelCommandGroup(
    shooter.PrepareShooter(0.5)
);

final Command AlageIntake = new ParallelCommandGroup(
    shooter.PrepareShooter(-0.5)
);

final Command ArmDown = new ParallelCommandGroup(
    arm.PrepareArm(-0.2)
);

final Command ArmUp = new ParallelCommandGroup(
    arm.PrepareArm(0.2)
);

final Command ClimbPrepare = new SequentialCommandGroup(
    new InstantCommand(() -> climber.runClimbMotor(-0.75)),
    new WaitCommand(3.0),
    new InstantCommand(() -> climber.climbStop())
    // new InstantCommand(() -> System.out.println("ClimbPrepare finished"))
);

final Command ClimbEndgame = new SequentialCommandGroup(
    new InstantCommand(() -> climber.runClimbMotor(0.5)),
    new WaitCommand(10),
    new InstantCommand(() -> climber.climbStop())
);

// Temporary commands to test climb motor
final Command tempClimbF = new SequentialCommandGroup(
    climber.PrepareClimber(-0.5)
);
final Command tempClimbB = new ParallelCommandGroup(
    climber.PrepareClimber(0.5)
);

    private void configureButtonBindings() {
        new JoystickButton(driverJoytick, OIConstants.kDriverResetGyroButtonIdx).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        final JoystickButton outtakeButton = new JoystickButton(xbox, 1);
        outtakeButton.whileTrue(ShootPiece);
        final JoystickButton intakeButton = new JoystickButton(xbox, 4);
        intakeButton.whileTrue(AlageIntake);
        final JoystickButton armDownButton = new JoystickButton(xbox, 5);
        armDownButton.whileTrue(ArmDown);
        final JoystickButton armUpButton = new JoystickButton(xbox, 6);
        armUpButton.whileTrue(ArmUp);
        final JoystickButton climbPrepareButton = new JoystickButton(xbox, 7);
        climbPrepareButton.onTrue(ClimbPrepare);
        final JoystickButton climbEndgameButton = new JoystickButton(xbox, 8);
        climbEndgameButton.onTrue(ClimbEndgame);

        // Temporary
        final JoystickButton tempP = new JoystickButton(xbox, 3);
        tempP.whileTrue(tempClimbB);
        final JoystickButton tempE = new JoystickButton(xbox, 2);
        tempE.whileTrue(tempClimbF);
    }

     public Command getAutonomousCommand() {

        // // 1. Create trajectory settings
        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        //         AutoConstants.kMaxSpeedMetersPerSecond,
        //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                 .setKinematics(DriveConstants.kDriveKinematics);

        // // 2. Generate trajectory
        // // Note: -y value is to the left (field relative)
        // // This sa,ple is an example of a figure 8 auto path
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         List.of(
        //                 new Translation2d(.7, -0.7),
        //                 new Translation2d(1.2, 0),
        //                 new Translation2d(.7, 0.7),
        //                 new Translation2d(0, 0),
        //                 new Translation2d(-.7, -0.7),
        //                 new Translation2d(-1.2, 0),
        //                 new Translation2d(-.7, 0.7)
        //                 ),
        //         new Pose2d(
        //        0, 0
        //         , Rotation2d.fromDegrees(0)),
        //         trajectoryConfig);      


        // // 3. Define PID controllers for tracking trajectory
        // PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        // PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        //         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // // 4. Construct command to follow trajectory
        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //         trajectory,
        //         swerveSubsystem::getPose,
        //         DriveConstants.kDriveKinematics,
        //         xController,
        //         yController,
        //         thetaController,
        //         swerveSubsystem::setModuleStates,
        //         swerveSubsystem);

        // // 5. Add some init and wrap-up, and return everything
        // return new SequentialCommandGroup(
        //         new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
        //         swerveControllerCommand,
        //         new InstantCommand(() -> swerveSubsystem.stopModules()));

        return new PathPlannerAuto("Test_Auto");
}

    public static Joystick getDriverJoytick() {
        return driverJoytick;
    }
}

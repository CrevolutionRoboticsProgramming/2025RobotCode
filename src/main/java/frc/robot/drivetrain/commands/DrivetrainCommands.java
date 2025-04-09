package frc.robot.drivetrain.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivetrain.CommandSwerveDrivetrain;


public class DrivetrainCommands {
    public static Command drive(double velocityX, double velocityY, double velocityRotational) {
        return new TeleopDrive(velocityX, velocityY, velocityRotational);
    }

    public static Command turn180InPlaceCommand() {
        return new Command() {
            private final PIDController thetaController = new PIDController(0.01, 0.0, 0);
            private double targetAngle;

            {
                addRequirements(CommandSwerveDrivetrain.getInstance());
                thetaController.enableContinuousInput(-180.0, 180.0);
            }

            @Override
            public void initialize() {
                double currentAngle = CommandSwerveDrivetrain.getInstance().getPigeon2().getAngle();
                targetAngle = Math.IEEEremainder(currentAngle + 180.0, 360.0);
                thetaController.setSetpoint(targetAngle);
            }

            @Override
            public void execute() {
                double currentAngle = CommandSwerveDrivetrain.getInstance().getPigeon2().getAngle();
                double rotationRate = thetaController.calculate(currentAngle);
                ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, rotationRate);
                SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
                            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
                CommandSwerveDrivetrain.getInstance().setControl(applyFieldSpeeds.withSpeeds(speeds));
            }

            @Override
            public boolean isFinished() {
                return thetaController.atSetpoint();
            }

            @Override
            public void end(boolean interrupted) {
                CommandSwerveDrivetrain.getInstance().stopSwerve();;
            }
        };
    }
}
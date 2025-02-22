package frc.robot.drivetrain2.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivetrain2.Drivetrain;
import frc.robot.drivetrain2.DrivetrainConfig.DriveConstants;


public class DrivetrainCommands {
    public static Command drive(double velocityX, double velocityY, double velocityRotational) {
        return new TeleopDrive(velocityX, velocityY, velocityRotational);
    }
}

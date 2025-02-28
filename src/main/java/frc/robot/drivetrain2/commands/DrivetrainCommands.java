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
    public static Command drive(Supplier<Translation2d> translationSupplier, DoubleSupplier rotationSupplier, double translationModifier,
                                 double rotationModifier, boolean isFieldOriented, Translation2d rotationOffset, boolean modeS, boolean modeA) {
        return new TeleopDrive(
            () -> translationSupplier.get().times(DriveConstants.MAX_SPEED).times(translationModifier),
            () -> Rotation2d.fromRadians(rotationSupplier.getAsDouble()).times(DriveConstants.MAX_ANGULAR_VELOCITY).times(rotationModifier),
            isFieldOriented,
            rotationOffset,
            modeS,
            modeA
        );
    }

    public static Command stopSwerve(Supplier<Translation2d> translationSupplier) {
        return drive(
            translationSupplier, 
            () -> 0.0,
            1.0,
            1.0,
            true,
            new Translation2d(0, 0),
            false,
            false
        );
    }

    public static Command drive(Supplier<Translation2d> translationSupplier, DoubleSupplier rotation) {
        return drive(translationSupplier, rotation, 1.0, 1.0, true, new Translation2d(0, 0), false, false);
    }

    public static Command driveSlowMode(Supplier<Translation2d> translationSupplier, DoubleSupplier rotation, boolean modeS, boolean modeA) {
        return drive(
            translationSupplier,
            rotation,
            DriveConstants.kSlowModeTranslationModifier,
            DriveConstants.kSlowModeRotationModifier,
            true,
            new Translation2d(0, 0),
            modeS,
            modeA
        );
    }
}

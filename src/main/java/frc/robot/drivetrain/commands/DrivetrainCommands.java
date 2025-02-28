package frc.robot.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.Command;


public class DrivetrainCommands {
    public static Command drive(double velocityX, double velocityY, double velocityRotational) {
        return new TeleopDrive(velocityX, velocityY, velocityRotational);
    }
}
package frc.robot.drivetrain.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.crevolib.io.JoystickConfig;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.drivetrain.TunerConstants;
import frc.robot.drivetrain2.Drivetrain;
import frc.robot.drivetrain2.DrivetrainConfig;
import frc.robot.drivetrain2.DrivetrainConfig.DriveConstants;
import frc.robot.drivetrain2.DrivetrainConfig.DriveConstants.*;
import static edu.wpi.first.units.Units.*;

public class TeleopDrive extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private double velocityX, velocityY, velocityRotational;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    /**
     *
     * @param translationSupplier translation demand, magnitude should be of interval [-1, 1], percent of max translational velocity
     * @param rotationSupplier interval from [-1, 1], percent of max angular velocity
     * @param isFieldRelative field relative or robot centric
     * @param rotationOffset offset for the robot's center of rotation
     */
    public TeleopDrive(double velocityX, double velocityY, double velocityRotational) {
        drivetrain = CommandSwerveDrivetrain.getInstance();
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.velocityRotational = velocityRotational;

        addRequirements(drivetrain);
    }


    @Override
    public void execute() {
        CommandSwerveDrivetrain.getInstance().applyRequest(() -> 
            drive.withVelocityX(velocityX) // Drive forward with negative Y (forward)
                .withVelocityY(velocityY) // Drive left with negative X (left)
                .withRotationalRate(velocityRotational) // Drive counterclockwise with negative X (left)
        );
        // System.out.println("LOL COmmand is BEING CALLED");
    }
}

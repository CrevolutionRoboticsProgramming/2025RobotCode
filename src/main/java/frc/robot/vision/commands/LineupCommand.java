package frc.robot.vision.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.vision.VisionConfig;
import frc.robot.RobotContainer;
import frc.robot.drivetrain.*;
import frc.robot.vision.PoseEstimatorSubsystem;

public class LineupCommand extends Command {

     double x;
     double y;
     double theta;
     LinearVelocity kLineupSpeed = MetersPerSecond.of(.1);

    private static final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
                                        VisionConfig.AlignmentConfig.MAX_ALIGN_TRANSLATION_VELOCITY.in(MetersPerSecond),
                                        VisionConfig.AlignmentConfig.MAX_ALIGN_TRANSLATION_ACCELERATION.in(MetersPerSecondPerSecond));
            
    private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                                        VisionConfig.AlignmentConfig.MAX_ALIGN_ANGULAR_VELOCITY.in(RadiansPerSecond),
                                        VisionConfig.AlignmentConfig.MAX_ALIGN_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));


    private static final ProfiledPIDController xDistanceController = new ProfiledPIDController(
                                        VisionConfig.AlignmentConfig.X_kP,
                                        VisionConfig.AlignmentConfig.X_kI,
                                        VisionConfig.AlignmentConfig.X_kD,
                                        TRANSLATION_CONSTRAINTS);

    private static final ProfiledPIDController yDistanceController = new ProfiledPIDController(
                                        VisionConfig.AlignmentConfig.Y_kP,
                                        VisionConfig.AlignmentConfig.Y_kI,
                                        VisionConfig.AlignmentConfig.Y_kD,
                                        TRANSLATION_CONSTRAINTS);

    private static final ProfiledPIDController thetaController = new ProfiledPIDController(
                                        VisionConfig.AlignmentConfig.THETA_kP,
                                        VisionConfig.AlignmentConfig.THETA_kI,
                                        VisionConfig.AlignmentConfig.THETA_kD,
                                        THETA_CONSTRAINTS);
    
    private static final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
                                        .withDriveRequestType(DriveRequestType.Velocity)
                                        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        

    private static Pose2d currentPose;
    private static Pose2d targetPose;
    private static boolean leftReefLineup;

    public LineupCommand(boolean left) {
        //set tolerances of all PID controllers
        xDistanceController.setTolerance(VisionConfig.AlignmentConfig.DISTANCE_TOLERANCE.in(Meters));
        xDistanceController.setIntegratorRange(-.15, .15);
        yDistanceController.setTolerance(VisionConfig.AlignmentConfig.LATERAL_TOLERANCE.in(Meters));
        yDistanceController.setIntegratorRange(-.15, .15);
        thetaController.enableContinuousInput(Units.degreesToRadians(-180), Units.degreesToRadians(180));

        currentPose = PoseEstimatorSubsystem.getInstance().getCurrentPose();
        targetPose = getTargetReefPose(left);

        xDistanceController.reset(0);
        yDistanceController.reset(0);
        thetaController.reset(0);

        leftReefLineup = left;

        addRequirements(CommandSwerveDrivetrain.getInstance());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        currentPose = PoseEstimatorSubsystem.getInstance().getCurrentPose();
        // ChassisSpeeds currentSpeeds = CommandSwerveDrivetrain.getInstance().getState().Speeds;
        x = currentPose.getTranslation().getX();
        y = currentPose.getTranslation().getY();
        // double Xvel = currentSpeeds.vxMetersPerSecond;
        // double Yvel = currentSpeeds.vyMetersPerSecond;
        double XOutput = 0.15 * kLineupSpeed.in(MetersPerSecond) * 0.95 * xDistanceController.calculate(x, targetPose.getX());
        double YOutput = 0.15 * kLineupSpeed.in(MetersPerSecond) * 0.95 * xDistanceController.calculate(y, targetPose.getY());
        double thetaOutput = 0.15 * RotationsPerSecond.of(0.75).in(RadiansPerSecond) * thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        if (DriverStation.getAlliance().get() == Alliance.Blue){
            CommandSwerveDrivetrain.getInstance().applyRequest(() -> 
                RobotContainer.drive.withVelocityX(-XOutput)
                     .withVelocityY(-YOutput)
                     .withRotationalRate(thetaOutput)
                     
            ).execute();
        } else {
            CommandSwerveDrivetrain.getInstance().applyRequest(() -> 
            RobotContainer.drive.withVelocityX(XOutput)
                     .withVelocityY(YOutput)
                     .withRotationalRate(-thetaOutput)   
            ).execute();
        }

        SmartDashboard.putString("Target Pose X", targetPose.toString());

    }
    

    @Override
    public boolean isFinished() {
        return xDistanceController.atSetpoint() && yDistanceController.atSetpoint() && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    private Pose2d getTargetReefPose(boolean left){ 
        ChassisSpeeds currentSpeeds = CommandSwerveDrivetrain.getInstance().getState().Speeds;
        double X = currentPose.getTranslation().getX() * currentSpeeds.vxMetersPerSecond * 0.1;
        double Y = currentPose.getTranslation().getY() * currentSpeeds.vyMetersPerSecond * 0.1;

        double intercept = Math.tan(Units.degreesToRadians(30))*4.5;
        double interceptRed = Math.tan(Units.degreesToRadians(30))*13;
        double slope = Math.tan(Units.degreesToRadians(30));

        var currAlliance = DriverStation.getAlliance();

        //TODO: map currentPose to a Reef Zone and then finding closest lineup based on heading
        //joey... what the actual god-loving heck is this

        if(currAlliance.get() == Alliance.Blue) {

            if (Y <= -X*slope + intercept+4 && Y >= X*slope - intercept+4 && left && X <= 4.5) {
            System.out.println("Ablue");
            return VisionConfig.AlignmentConfig.Ablue;
            } else if (Y <= -X*slope + intercept+4 && Y >= X*slope - intercept+4 && !left && X <= 4.5) {
            System.out.println("Bblue");
            return VisionConfig.AlignmentConfig.Bblue;
            } else if (Y <= X*slope - intercept+4 && X <= 4.5 && left) {
            System.out.println("Cblue");
            return VisionConfig.AlignmentConfig.Cblue;
            } else if (Y <= X*slope - intercept+4 && X <= 4.5 && !left) {
            System.out.println("Dblue");
            return VisionConfig.AlignmentConfig.Dblue;
            } else if (Y <= -X*slope + intercept+4 && X >= 4.5 && !left) {
            System.out.println("Eblue");
            return VisionConfig.AlignmentConfig.Eblue;
            } else if (Y <= -X*slope + intercept+4 && X >= 4.5 && left) {
            System.out.println("Fblue");
            return VisionConfig.AlignmentConfig.Fblue;
            } else if (Y >= -X*slope + intercept+4 && Y <= X*slope - intercept+4 && !left && X >= 4.5) {
            System.out.println("Gblue");
            return VisionConfig.AlignmentConfig.Gblue;
            } else if (Y >= -X*slope + intercept+4 && Y <= X*slope - intercept+4 && left && X >= 4.5) {
            System.out.println("Hblue");
            return VisionConfig.AlignmentConfig.Hblue;
            } else if (Y >= X*slope - intercept+4 && X >= 4.5 && !left) {
            System.out.println("Iblue");
            return VisionConfig.AlignmentConfig.Iblue;
            } else if (Y >= X*slope - intercept+4 &&  X >= 4.5 && left) {
            System.out.println("Jblue");
            return VisionConfig.AlignmentConfig.Jblue;
            } else if (Y >= -X*slope + intercept+4 && X <= 4.5 && left) {
            System.out.println("Kblue");
            return VisionConfig.AlignmentConfig.Kblue;
            } else if (Y >= -X*slope + intercept+4 && X <= 4.5 && !left) {
            System.out.println("Lblue");
            return VisionConfig.AlignmentConfig.Lblue;
            } else {
            System.out.println("error");
            return VisionConfig.AlignmentConfig.Error;
            }
        }
        else if(currAlliance.get() == Alliance.Red) {
            if (Y <= -X*slope + interceptRed+4 && Y >= X*slope - interceptRed+4 && !left && X <= 13.0) {
                System.out.println("Gred");
                return VisionConfig.AlignmentConfig.Gred;
                } else if (Y <= -X*slope + interceptRed+4 && Y >= X*slope - interceptRed+4 && left && X <= 13.0) {
                System.out.println("Hred");
                return VisionConfig.AlignmentConfig.Hred;
                } else if (Y <= X*slope - interceptRed+4 && X <= 13.0 && !left) {
                System.out.println("Ired");
                return VisionConfig.AlignmentConfig.Ired;
                } else if (Y <= X*slope - interceptRed+4 && X <= 13.0 && left) {
                System.out.println("Jred");
                return VisionConfig.AlignmentConfig.Jred;
                } else if (Y <= -X*slope + interceptRed+4 && X >= 13.0 && left) {
                System.out.println("Kred");
                return VisionConfig.AlignmentConfig.Kred;
                } else if (Y <= -X*slope + interceptRed+4 && X >= 13.0 && !left) {
                System.out.println("Lred");
                return VisionConfig.AlignmentConfig.Lred;
                } else if (Y >= -X*slope + interceptRed+4 && Y <= X*slope - interceptRed+4 && left && X >= 13.0) {
                System.out.println("Ared");
                return VisionConfig.AlignmentConfig.Ared;
                } else if (Y >= -X*slope + interceptRed+4 && Y <= X*slope - interceptRed+4 && !left && X >= 13.0) {
                System.out.println("Bred");
                return VisionConfig.AlignmentConfig.Bred;
                } else if (Y >= X*slope - interceptRed+4 && X >= 13.0 && left) {
                System.out.println("Cred");
                return VisionConfig.AlignmentConfig.Cred;
                } else if (Y >= X*slope - interceptRed+4 &&  X >= 13.0 && !left) {
                System.out.println("Dred");
                return VisionConfig.AlignmentConfig.Dred;
                } else if (Y >= -X*slope + interceptRed+4 && X <= 13.0 && !left) {
                System.out.println("Ered");
                return VisionConfig.AlignmentConfig.Ered;
                } else if (Y >= -X*slope + interceptRed+4 && X <= 13.0 && left) {
                System.out.println("Fred");
                return VisionConfig.AlignmentConfig.Fred;
                } else {
                System.out.println("error");
                return VisionConfig.AlignmentConfig.Error;
                }
        }
        else {
            System.out.println("Error: Invalid Alliance (Lineup Command)");
        }

        System.out.println("Error: no suitable target pose found (Lineup Command)");
        return new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
    }
}


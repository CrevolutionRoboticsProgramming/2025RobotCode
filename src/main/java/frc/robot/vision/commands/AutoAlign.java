package frc.robot.vision.commands;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.vision.VisionConfig.*;
import static frc.robot.vision.VisionConfig.AlignmentConfig.*;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.vision.PoseEstimatorSubsystem;
import frc.robot.vision.VisionConfig;



public class AutoAlign extends Command {
    private double x;
    private double y;
    private LinearVelocity kLineupSpeed = MetersPerSecond.of(.1);
    private PhotonCamera leftCam = new PhotonCamera(CAM_NAMES[0]); 
    private int tagID;

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

                                        
                                        
    private static Pose2d currentPose;
    private static Pose2d targetPose;
    private static boolean commandRan = false;

    public AutoAlign(boolean left) {
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

        addRequirements(CommandSwerveDrivetrain.getInstance());
    }

    
    public double getDistanceFromTag(Pose2d goalPose) {
        double xDiff = goalPose.getX() - currentPose.getX();
        double yDiff = goalPose.getY() - currentPose.getY();
        double distance = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
        return distance;
    }

    private Pose2d getTargetReefPose(boolean left){
        var photonResult = leftCam.getAllUnreadResults();
        var currAlliance = DriverStation.getAlliance();
        
        for (var result : photonResult){
            if (result.hasTargets()){
                tagID = result.getBestTarget().fiducialId;
            }
        }

        if (currAlliance.get() == Alliance.Blue){
            // IF(LEFT)
            // {
            //     CLOSESTDIST = NULL;
            //     TARGETPOSE = A;
            //     TEMPdIST = DISTANCE(CURPOS, TARGETPOSE);
            //     CLOSESTDIST = TEMPDIST;
            //     POSE = A

            //     TARGETPOSE = C;
            //     TEMPdIST = DISTANCE(CURPOS, TARGETPOSE)
            //     IF(TEMPDIST < CLOSESTDIST)
            //         POSE = C;
                
            // }
            if (left) {
                double min = getDistanceFromTag(Ablue);
                targetPose = Ablue;
                
                if (min > getDistanceFromTag(Cblue)){
                    min = getDistanceFromTag(Cblue);
                    targetPose = Cblue;
                } 
                if (min > getDistanceFromTag(Kblue)) {
                    min = getDistanceFromTag(Kblue);
                    targetPose = Kblue;
                }
                if (min > getDistanceFromTag(Iblue)) {
                    min = getDistanceFromTag(Iblue);
                    targetPose = Iblue;
                } 
                if (min > getDistanceFromTag(Gblue)) {
                    min = getDistanceFromTag(Gblue);
                    targetPose = Gblue;
                } 
                if (min > getDistanceFromTag(Eblue)) {
                    min = getDistanceFromTag(Eblue);
                    targetPose = Eblue;
                }
            } else {
                double min = getDistanceFromTag(Bblue);
                targetPose = Ablue;
                
                if (min > getDistanceFromTag(Dblue)){
                    min = getDistanceFromTag(Dblue);
                    targetPose = Dblue;
                } 
                if (min > getDistanceFromTag(Lblue)) {
                    min = getDistanceFromTag(Lblue);
                    targetPose = Lblue;
                }
                if (min > getDistanceFromTag(Jblue)) {
                    min = getDistanceFromTag(Jblue);
                    targetPose = Jblue;
                } 
                if (min > getDistanceFromTag(Hblue)) {
                    min = getDistanceFromTag(Hblue);
                    targetPose = Hblue;
                } 
                if (min > getDistanceFromTag(Fblue)) {
                    min = getDistanceFromTag(Fblue);
                    targetPose = Fblue;
                }
            }

            // //DISTANCE FROM 
            // if (targetPose == Cblue){
            //     if (left) {
            //         return Cblue;
            //     } else if (!left) {
            //         return Dblue;
            //     }
            // } else if (getDistanceFromTag(Ablue) < getDistanceFromTag(Kblue)) {
            //     if (left) {
            //         return Ablue;
            //     } else if (!left) {
            //         return Bblue;
            //     }
            // } else if (getDistanceFromTag(Kblue) < getDistanceFromTag(Iblue)) {
            //     if (left) {
            //         return Kblue;
            //     } else if (!left) {
            //         return Lblue;
            //     }
            // } else if (getDistanceFromTag(Iblue) < getDistanceFromTag(Gblue)) {
            //     if (left) {
            //         return Iblue;
            //     } else if (!left) {
            //         return Jblue;
            //     }
            // } else if (getDistanceFromTag(Gblue) < getDistanceFromTag(Eblue)) {
            //     if (left) {
            //         return Gblue;
            //     } else if (!left) {
            //         return Hblue;
            //     }
            // } else if (getDistanceFromTag(Eblue) < getDistanceFromTag(Cblue)) {
            //     if (left) {
            //         return Eblue;
            //     } else if (!left) {
            //         return Fblue;
            //     }
            // }

        } else if (currAlliance.get() == Alliance.Red){
            if (left) {
                double min = getDistanceFromTag(Ared);
                targetPose = Ared;
                
                if (min > getDistanceFromTag(Cred)){
                    min = getDistanceFromTag(Cred);
                    targetPose = Cred;
                } 
                if (min > getDistanceFromTag(Kred)) {
                    min = getDistanceFromTag(Kred);
                    targetPose = Kblue;
                }
                if (min > getDistanceFromTag(Ired)) {
                    min = getDistanceFromTag(Ired);
                    targetPose = Ired;
                } 
                if (min > getDistanceFromTag(Gred)) {
                    min = getDistanceFromTag(Gred);
                    targetPose = Gred;
                } 
                if (min > getDistanceFromTag(Ered)) {
                    min = getDistanceFromTag(Ered);
                    targetPose = Ered;
                }
            } else {
                double min = getDistanceFromTag(Bred);
                targetPose = Ablue;
                
                if (min > getDistanceFromTag(Dred)){
                    min = getDistanceFromTag(Dred);
                    targetPose = Dred;
                } 
                if (min > getDistanceFromTag(Lred)) {
                    min = getDistanceFromTag(Lred);
                    targetPose = Lred;
                }
                if (min > getDistanceFromTag(Jred)) {
                    min = getDistanceFromTag(Jred);
                    targetPose = Jred;
                } 
                if (min > getDistanceFromTag(Hred)) {
                    min = getDistanceFromTag(Hred);
                    targetPose = Hred;
                } 
                if (min > getDistanceFromTag(Fred)) {
                    min = getDistanceFromTag(Fred);
                    targetPose = Fred;
                }
            }

        } else if (currAlliance.get() != Alliance.Red && currAlliance.get() != Alliance.Blue){
            System.out.println("ERROR: There's no such thing as purple alliance");
        } 

        System.out.println("Error: no suitable target pose found (Lineup Command)");
        return targetPose;

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
        
        commandRan = true;
        SmartDashboard.putBoolean("command ran", commandRan);
        SmartDashboard.putString("Target Pose X", targetPose.toString());
        SmartDashboard.putBoolean("X at target", xDistanceController.atSetpoint());
        SmartDashboard.putBoolean("Y at target", yDistanceController.atSetpoint());
        SmartDashboard.putBoolean("Theta at target", thetaController.atSetpoint());
        //SmartDashboard.putNumber("TAG ID", tagID);

    }

    @Override
    public boolean isFinished() {
        return commandRan;
        // return xDistanceController.atSetpoint() && yDistanceController.atSetpoint() && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        
    }


}

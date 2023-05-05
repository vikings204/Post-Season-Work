package frc.robot.util;
//package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.nio.file.Path;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.util.Units;


public class generateTrajectory {
    
    //private final Swerve s_Swerve;
  private boolean complete = false;
  private double target_angle;
  private double current_angle;
  Translation2d target;
  Translation2d current;
  private Timer timer = new Timer();
  private double timeout;



public PathPlannerTrajectory getTrajectory(Translation2d current1, Translation2d target1, Rotation2d cr1, Rotation2d r1){
    current_angle = cr1.getDegrees();
    target_angle = r1.getDegrees();//degrees;
    target = target1;
    current = current1;
    System.out.println("Current Pose X: "+current1.getX());
    System.out.println("Current Pose Y: "+current1.getY());
    System.out.println("Target Pose X: "+target1.getX());
    System.out.println("Target Pose y: "+target1.getX());
    return PathPlanner.generatePath(
        new PathConstraints(1.5, 3),
        
        new PathPoint(
            current,
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(current_angle)),  
        new PathPoint(
              target,
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(target_angle)));
}
}

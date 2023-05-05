package frc.robot.commands;
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

public class DriveToPoint extends CommandBase {

  private final Swerve s_Swerve;
  private boolean complete = false;
  private double target_angle;
  private double current_angle;
  Translation2d target;
  Translation2d current;
  private Timer timer = new Timer();
  private double timeout;

  public DriveToPoint(Swerve subsystem, Translation2d current1, Translation2d target1, Rotation2d cr1, Rotation2d r1) {
    s_Swerve = subsystem;
    current_angle = cr1.getDegrees();
    target_angle = r1.getDegrees();//degrees;
    target = target1;
    current = current1;
    //timeout = timeoutS;
    addRequirements(subsystem);


  }

  @Override
  public void initialize() {
  
    complete = false;
  }

  @Override
  public void execute() {
    PathPlannerTrajectory trajectory1 =
    PathPlanner.generatePath(
        new PathConstraints(1.5, 3),
        
        new PathPoint(
            current,
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(current_angle)),  
        new PathPoint(
              target,
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(target_angle)));
    PPSwerveControllerCommand swerveControllerCommand1 =
    new PPSwerveControllerCommand(
        trajectory1,
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
        s_Swerve::setModuleStates,
        s_Swerve);
  }

  @Override
  public void end(boolean inturrupted) {
    s_Swerve.drive(new Translation2d(0, 0), 0, false, true);
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return complete;
  }
}

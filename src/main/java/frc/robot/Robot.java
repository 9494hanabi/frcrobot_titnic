package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

// PhotonVision のインポート
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// PathPlanner のインポート
import java.util.List;

 
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder.TriFunction;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.NamedCommands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.commands.PathfindThenFollowPath;

import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

public class Robot extends TimedRobot {

    // ========= モーター関係 =========
    private PWMSparkMax leftMotor1, leftMotor2, rightMotor1, rightMotor2;
    private Joystick joystick;

    private final int leftMotor1Port = 2;
    private final int leftMotor2Port = 3;
    private final int rightMotor1Port = 5;
    private final int rightMotor2Port = 4;
    private final int joystickPort = 0;

    // ========= PhotonVision 関係 =========
    // private PhotonCamera camera;
    // private final double cameraHeight = 0.27;
    // private final double targetHeight = 1.0;
    // private final double cameraPitch = Math.toRadians(5);
    // private final double desiredDistance = 1.0;
    // private final double kP_forward = 0.5;
    // private final double kP_turn = 0.02;

    // ========= PathPlanner 関係 =========
    private Timer timer;
    private PathPlannerTrajectory trajectory;
    private final double kTrackWidth = 0.5;  // ロボットのトラック幅（仮値。実際のロボットに合わせて調整すること）

    @Override
    public void robotInit() {
        // ========== モーター初期化 ==========
        leftMotor1 = new PWMSparkMax(leftMotor1Port);
        leftMotor2 = new PWMSparkMax(leftMotor2Port);
        rightMotor1 = new PWMSparkMax(rightMotor1Port);
        rightMotor2 = new PWMSparkMax(rightMotor2Port);

        joystick = new Joystick(joystickPort);

        // 右側モーターを反転
        rightMotor1.setInverted(true);
        rightMotor2.setInverted(true);

        // ========== カメラ初期化 ==========
        // camera = new PhotonCamera("Arducam_OV9281_USB_Camera (1) (2)");

        // ========== PathPlanner 用タイマー ==========
        timer = new Timer();
    }

    @Override
    public void teleopPeriodic() {
        double leftSpeed = joystick.getRawAxis(5);  // 左スティックY軸
        double rightSpeed = joystick.getRawAxis(1);  // 右スティックY軸

        leftMotor1.set(limitSpeedLeft(leftSpeed));
        leftMotor2.set(limitSpeedLeft(leftSpeed));
        rightMotor1.set(limitSpeedRight(rightSpeed) * 2);
        rightMotor2.set(limitSpeedRight(rightSpeed) * 2);

        SmartDashboard.putNumber("Left Speed", leftSpeed);
        SmartDashboard.putNumber("Right Speed", rightSpeed);
        SmartDashboard.putString("Mode", "Teleop");
    }

    @Override
    public void autonomousInit() {
        // PathPlanner の "Example Path" をロードし、タイマーをリセット
        trajectory = AutonomousPaths.Plactice;
        timer.reset();
        timer.start();
    }

    @Override
    public void autonomousPeriodic() {
        // ===== 元の PhotonVision によるターゲット追従コード =====
        /*
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            double targetPitchDeg = target.getPitch();
            double targetYawDeg = target.getYaw();
            double targetPitchRad = Math.toRadians(targetPitchDeg);
            double currentDistance = (targetHeight - cameraHeight)
                    / Math.tan(cameraPitch + targetPitchRad);

            double distanceError = desiredDistance - currentDistance;
            double forwardSpeed = kP_forward * distanceError;
            double turnSpeed = kP_turn * (targetYawDeg);

            double leftOutput = forwardSpeed - turnSpeed;
            double rightOutput = forwardSpeed + turnSpeed;

            leftOutput = limitSpeedLeft(leftOutput);
            rightOutput = limitSpeedRight(rightOutput);

            leftMotor1.set(leftOutput);
            leftMotor2.set(leftOutput);
            rightMotor1.set(rightOutput);
            rightMotor2.set(rightOutput);

            SmartDashboard.putNumber("Auto Distance", currentDistance);
            SmartDashboard.putNumber("Auto Yaw (deg)", targetYawDeg);
            SmartDashboard.putNumber("Auto Left Output", leftOutput);
            SmartDashboard.putNumber("Auto Right Output", rightOutput);
            SmartDashboard.putString("Autonomous Status", "Maintaining Distance");
        } else {
            stopMotors();
            SmartDashboard.putString("Autonomous Status", "No target - Stopped");
        }
        */

        // ===== PathPlanner の "Example Path" に沿った自律走行コード =====
        double currentTime = timer.get();
        if (currentTime > trajectory.getTotalTimeSeconds()) {
            stopMotors();
            
            SmartDashboard.putString("Autonomous Status", "Path Completed");
            return;
        }

        // 現在の時刻における目標状態を取得
        PathPlannerTrajectoryState desiredState = trajectory.sample(currentTime);
        // desiredState.velocity は直進速度 (m/s)、 desiredState.fieldSpeeds.omegaRadiansPerSecondは曲率 (1/m) を表す
        double angularVelocity = desiredState.fieldSpeeds.vxMetersPerSecond * desiredState.fieldSpeeds.omegaRadiansPerSecond;
        double linearVelocity =desiredState.fieldSpeeds.vxMetersPerSecond;

        // 差動ドライブ方式で左右のホイール速度を算出
        double leftOutput = linearVelocity - angularVelocity * kTrackWidth / 2;
        double rightOutput = linearVelocity + angularVelocity * kTrackWidth / 2;

        leftOutput = limitSpeedLeft(leftOutput);
        rightOutput = limitSpeedRight(rightOutput);

        leftMotor1.set(leftOutput);
        leftMotor2.set(leftOutput);
        rightMotor1.set(rightOutput);
        rightMotor2.set(rightOutput);

        SmartDashboard.putNumber("Trajectory Time", currentTime);
        SmartDashboard.putNumber("Desired Velocity", linearVelocity);
        SmartDashboard.putNumber("Left Output", leftOutput);
        SmartDashboard.putNumber("Right Output", rightOutput);
        SmartDashboard.putString("Autonomous Status", "Following Example Path");
    }

    // モーター出力を -1.0 ～ 1.0 に制限する
    private double limitSpeedLeft(double speed) {
        return Math.max(-1.0, Math.min(1.0, speed));
    }

    private double limitSpeedRight(double speed) {
        return Math.max(-0.25, Math.min(0.25, speed));
    }

    // モーター停止用ユーティリティ
    private void stopMotors() {
        leftMotor1.set(0);
        leftMotor2.set(0);
        rightMotor1.set(0);
        rightMotor2.set(0);
    }

    //======PathPlannerのファイル名をここに入れる======
    public PathPlannerAuto getAutonomousCommand(String pathName)
    {
        return new PathPlannerAuto("paths/Plactice");
    }
    public class AutonomousPaths{
        public static final PathPlannerTrajectory Plactice = 
            PathPlanner.loadPath("paths/Plactice", new PathConstraints(2.0,2.0, 0, 0));
    }
}
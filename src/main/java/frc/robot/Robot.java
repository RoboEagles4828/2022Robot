// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;


public class Robot extends TimedRobot {

  DriveTrain dt = new DriveTrain();
  double xSpeed = 0;
  double zRotation = 0;

  Pose2d pose = new Pose2d();
  Trajectory trajTest = new Trajectory();
  Trajectory threeBall = new Trajectory();
  // TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(Distances.max_vel), Units.feetToMeters(Distances.max_acc)).setKinematics(dt.kin);
  DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(dt.getFeedforward(), dt.kin, 12);
  
  ChassisSpeeds auto_chassis_speeds = new ChassisSpeeds();
  DifferentialDriveWheelSpeeds auto_speeds = new DifferentialDriveWheelSpeeds();
  RamseteController controller = new RamseteController();  

  Shooter shooter = new Shooter(Ports.shooter_port);
  Shooter shooter_back = new Shooter(Ports.shooter_back_port);
  boolean manual_shooter = false;
  double shooter_speed = 0;
  double shooter_back_speed = 0;
  boolean detected = false;

  Conveyor conveyor = new Conveyor();
  boolean manual_conveyor = false;
  double conveyor_speed = 0;

  Intake intake = new Intake();
  double intake_speed = 0;

  Climber climber = new Climber();
  double left_climber_speed = 0;
  double right_climber_speed = 0;
  boolean left_can_climb = true;//Hall effects allow to climb up on left
  boolean right_can_climb = true;//Hall effects allow to climb up on right
  int state = 0;//0 is both, 1 is left, 2 is right

  DigitalInput prox_lower = new DigitalInput(Ports.proxy_lower);
  DigitalInput prox_upper = new DigitalInput(Ports.proxy_upper);
  DigitalInput hall_left = new DigitalInput(Ports.hall_left);
  DigitalInput hall_right = new DigitalInput(Ports.hall_right);

  Servo intake_servo = new Servo(Ports.intake_servo_port);
  PWM shooter_servo_0 = new PWM(Ports.shooter_servo_0_port);
  PWM shooter_servo_1 = new PWM(Ports.shooter_servo_1_port);

  Joystick joystick_0 = new Joystick(Ports.joystick_0_port);
  Joystick joystick_1 = new Joystick(Ports.joystick_1_port);
  boolean climber_mode = false;

  AHRS navx = new AHRS();
  double starting_angle = 0;

  DifferentialDriveWheelSpeeds teleop_speeds = new DifferentialDriveWheelSpeeds();
  
  static final String DefaultAuto = "Default";
  static final String BasicAuto = "DriveShoot";
  static final String DriveAuto = "Drive";
  static final String WallToBallAuto = "WallToBallAuto";//Climbing area is "top" part
  static final String TarmacToBallAuto = "TarmacToBallAuto";
  static final String Shoot = "ShootAuto";
  static final String TurnRight = "TurnRight";
  static final String TurnLeft = "TurnLeft";
  static final String Triangle = "Triangle";
  static final String ThreeBall = "ThreeBall";
  static final String ThreeBallAuto = "ThreeBallAuto";
  static final String TrajectoryAuto = "TrajectoryAuto";
  SendableChooser<String> chooser = new SendableChooser<>();
  String autoSelected;
  Timer timer = new Timer();
  double starting_pos = 0;
  double start_right_pos = 0;
  int auto_state = 0;
  double auto_start_time = 0;//Only used for auto shooting
  double start_time = 0;//starting time for auto and recycled for auto shoot time
  boolean first = true;

  double stop_time = 0;//stop drive time
  double conveyor_index_time = 0;
  boolean is_spacing = false;

  double spoofX = 0;
  double spoofY = 0;

  NetworkTableEntry robotX, robotY, robotHeading, pathX, pathY, isFollowingPath;

  @Override
  public void robotInit(){
    chooser.setDefaultOption("Default Auto", DefaultAuto);
    chooser.addOption("Basic Auto", BasicAuto);
    chooser.addOption("Drive Auto", DriveAuto);
    chooser.addOption("TarmacToBallAuto", TarmacToBallAuto);
    chooser.addOption("Wall To Ball Auto", WallToBallAuto);
    chooser.addOption("Shoot Auto", Shoot);
    chooser.addOption("Turn Right", TurnRight);
    chooser.addOption("Turn Left", TurnLeft);
    chooser.addOption("Triangle", Triangle);
    chooser.addOption("Three Ball", ThreeBall);
    chooser.addOption("Three Ball Auto", ThreeBallAuto);
    chooser.addOption("Trajectory test", TrajectoryAuto);
    SmartDashboard.putData("Auto Choices:", chooser);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable dashboard_info = inst.getTable("Live_Dashboard");
    robotX = dashboard_info.getEntry("robotX");
    robotY = dashboard_info.getEntry("robotY");
    robotHeading = dashboard_info.getEntry("robotHeading");

    pathX = dashboard_info.getEntry("pathX");
    pathY = dashboard_info.getEntry("pathY");
    isFollowingPath = dashboard_info.getEntry("isFollowingTrajectory");
    
    //CameraServer.startAutomaticCapture();
    //CameraServer.startAutomaticCapture();//Call twice to automatically create both cameras and have them as optional displays
    //dt.front_left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);//Defaults to integrated sensor, this is quadrature    
    //navx.calibrate();
  }

  @Override
  public void autonomousInit(){
    System.out.println(1.0/dt.convertMeters(1));
    autoSelected=chooser.getSelected();
    System.out.println("Auto selected: " + autoSelected);
    timer.reset();
    timer.start();
    starting_pos=dt.front_left.getSelectedSensorPosition();
    start_right_pos=dt.front_right.getSelectedSensorPosition();
    intake_servo.set(1);
    auto_state=0;
    conveyor_speed=0;
    intake_speed=0;
    shooter_speed=0;
    xSpeed=0;
    zRotation=0;
    stop_time=0;
    navx.reset();
    dt.navx.reset();
    //loading PathWeaver json files from deploy directory
    Path ThreeBallPath = Filesystem.getDeployDirectory().toPath();
    try {
      ThreeBallPath = Filesystem.getDeployDirectory().toPath().resolve("paths/output/3ball.wpilib.json");
      System.out.println("                              " + Filesystem.getDeployDirectory().getName());
      threeBall = TrajectoryUtil.fromPathweaverJson(ThreeBallPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + ThreeBallPath, ex.getStackTrace());
    }

    Path trajectoryPath = Filesystem.getDeployDirectory().toPath();
    try {
     trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/output/Unnamed.wpilib.json");
     System.out.println("                              " + Filesystem.getDeployDirectory().getName());
     trajTest = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryPath, ex.getStackTrace());
    }
    switch(autoSelected){
      case TrajectoryAuto:
        dt.odom.resetPosition(new Pose2d(new Translation2d(Units.metersToFeet(trajTest.getInitialPose().getX()), Units.metersToFeet(trajTest.getInitialPose().getY())), trajTest.getInitialPose().getRotation()), trajTest.getInitialPose().getRotation());
        break;
      case ThreeBallAuto:
        dt.odom.resetPosition(new Pose2d(new Translation2d(Units.metersToFeet(threeBall.getInitialPose().getX()), Units.metersToFeet(threeBall.getInitialPose().getY())), threeBall.getInitialPose().getRotation()), threeBall.getInitialPose().getRotation());
        break;
    }

  }

  @Override
  public void autonomousPeriodic(){

    pose = dt.odom.update(dt.getHeading(), dt.convertMeters(dt.front_left.getSelectedSensorPosition()-starting_pos), dt.convertMeters(dt.front_right.getSelectedSensorPosition()-starting_pos));
    Trajectory.State goal = new Trajectory.State();

    //If ball detected make conveyor move to pull it
    if(prox_lower.get()){
      conveyor_speed=Speeds.index_conveyor_speed;
      conveyor_index_time=timer.get();
    }else if(!manual_conveyor&&timer.get()-conveyor_index_time>0.25){//If ball is not detected, conveyor will stop
      conveyor_speed=0;
    }

    //If ball is detected by upper proxy stop conveyor
    if(!prox_upper.get()&&!manual_shooter){
      conveyor_speed=0;
    }

    //Determine what auto to run

    switch(autoSelected){
      case TrajectoryAuto:
        switch(auto_state){
          case 0:
          //intake_speed = Speeds.auto_intake_speed;f
          goal = trajTest.sample(timer.get());
          // if(timer.get() > 0.1){
          //   pose = trajTest.sample(timer.get()-0.1).poseMeters;
          // }
          auto_chassis_speeds = controller.calculate(pose, goal);
          System.out.println(auto_chassis_speeds);
          robotX.setDouble(Units.metersToFeet(pose.getX()));
          robotY.setDouble(Units.metersToFeet(pose.getY()));
          robotHeading.setDouble(pose.getRotation().getRadians());
          pathX.setDouble(Units.metersToFeet(goal.poseMeters.getX()));
          pathY.setDouble(Units.metersToFeet(goal.poseMeters.getY()));
          isFollowingPath.setBoolean(true);
          System.out.println("X: " + pose.getX() + ", Y: " + pose.getY() + " - rotation " + pose.getRotation().getDegrees());
          auto_speeds = dt.kin.toWheelSpeeds(auto_chassis_speeds);
          System.out.println(auto_speeds);
          if(timer.get() >= trajTest.getTotalTimeSeconds()){
            auto_speeds = new DifferentialDriveWheelSpeeds(0, 0);
            intake_speed = 0;
            auto_state++;
          }
          break;
        }
      break;
      case Triangle:
        switch(auto_state){
          case 0:
            if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<96*Distances.encoder_ratio){
              xSpeed=0.3;
            }else{
              xSpeed=0;
              navx.reset();
              auto_state++;
            }
            break;
          case 1:
            if(navx.getAngle()%360<90-Distances.turning_error){
              zRotation=Speeds.auto_turn_speed;
              //xSpeed=0.3;//For Curvature
            }else{
              zRotation=0;
              auto_state++;
              starting_pos=dt.front_left.getSelectedSensorPosition();
            }
            break;
          case 2:
            if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<72*Distances.encoder_ratio){
              xSpeed=0.3;
            }else{
              xSpeed=0;
              auto_state++;
              navx.reset();
            }
            break;
          case 3:
            if(navx.getAngle()%360<90-Distances.turning_error){
              zRotation=Speeds.auto_turn_speed;
            }else{
              zRotation=0;
              auto_state++;
              navx.reset();
              starting_pos=dt.front_left.getSelectedSensorPosition();
            }
            break;
          case 4:
            if(navx.getAngle()%360<60-Distances.turning_error){
              zRotation=Speeds.auto_turn_speed;
            }else{
              zRotation=0;
            }
            if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<120*Distances.encoder_ratio){
              xSpeed=0.3;
            }else{
              xSpeed=0;
            }
            break;
        }
        break;

      case ThreeBallAuto:
          switch(auto_state){
            case 0:
              if(timer.get()<Times.wall_shoot_time){
                shooter_speed=Speeds.wall_shooter_speed;
                manual_shooter=true;
                if(timer.get()>Times.conveyor_first_delay){
                  conveyor_speed=Speeds.conveyor_shoot_speed;
                  manual_conveyor=true;
                }
              }else{
                conveyor_speed=0;
                manual_conveyor=false;
                auto_state++;
              }
              break;
            case 1:
              intake_speed = Speeds.auto_intake_speed;
              goal = threeBall.sample(timer.get()-Times.wall_shoot_time);
              // if(timer.get()-Times.wall_shoot_time > 0.01){
              //   pose = threeBall.sample(timer.get()-Times.wall_shoot_time-0.01).poseMeters;
              // }
              System.out.println(timer.get()-Times.wall_shoot_time);
              auto_chassis_speeds = controller.calculate(pose, goal);
              System.out.println(auto_chassis_speeds);
              System.out.println(Double.toString(pose.getX()) + ", " + pose.getY() + " - rotation " + pose.getRotation().getDegrees());
              auto_speeds = dt.kin.toWheelSpeeds(auto_chassis_speeds);
              robotX.setDouble(Units.metersToFeet(pose.getX()));
              robotY.setDouble(Units.metersToFeet(pose.getY()));
              robotHeading.setDouble(pose.getRotation().getRadians());
              pathX.setDouble(Units.metersToFeet(goal.poseMeters.getX()));
              pathY.setDouble(Units.metersToFeet(goal.poseMeters.getY()));
              isFollowingPath.setBoolean(true);
              System.out.println(auto_speeds);
              if(timer.get()-Times.wall_shoot_time >= threeBall.getTotalTimeSeconds()){
                auto_speeds = new DifferentialDriveWheelSpeeds(0.0, 0.0);
                intake_speed = 0;
                auto_state++;
              }
              break;
            case 2:
              if(timer.get()-Times.wall_shoot_time-threeBall.getTotalTimeSeconds() < Times.wall_shoot_time){
                if(first){
                  conveyor_speed=Speeds.conveyor_shoot_speed;
                  manual_conveyor=true;
                }else{
                  if(timer.get()-start_time>=Times.conveyor_delay){
                    conveyor_speed=Speeds.conveyor_shoot_speed;
                    manual_conveyor=true;
                  }else{
                    conveyor_speed=0;
                    manual_conveyor=false;
                  }
                }
                if(!detected&&!prox_upper.get()){//Detects a ball as first ball
                  detected=true;
                }else if(detected&&prox_upper.get()){//Not at proxy and has been detected, aka the first ball left
                  start_time=timer.get();
                  detected=false;
                  first=false;
                }
                manual_shooter = true;
                break;
          }
        }
        break;

        case TarmacToBallAuto:
          switch(auto_state){
            case 0:
              if(timer.get()>Times.intake_drop_time){
                auto_state++;
              }
              break;
            case 1:
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Distances.tarmac_to_ball*Distances.encoder_ratio){
                xSpeed=Speeds.auto_drive_speed;
                intake_speed=Speeds.auto_intake_speed;
              }else{
                auto_state++;
                starting_pos=dt.front_left.getSelectedSensorPosition();
              }
              break;
            case 2://Drive back to shoot and start reving up shooter
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Distances.ball_to_wall*Distances.encoder_ratio){
                xSpeed=-Speeds.auto_drive_speed;
                shooter_speed=Speeds.wall_shooter_speed;
              }else{
                auto_state++;
                stop_time=timer.get();
              }
              break;
            case 3://Stop
              if(timer.get()-stop_time<Times.stop_dt_time){
                xSpeed=0.5;
              }else{
                xSpeed=0;
                auto_state++;
                intake_speed=0;
                start_time=timer.get();
              }
              break;
            case 4://Shoot
              if(timer.get()-start_time<Times.wall_shoot_time){
                shooter_speed=Speeds.wall_shooter_speed;
                  if(first){
                    conveyor_speed=Speeds.conveyor_shoot_speed;
                    manual_conveyor=true;
                  }else{
                    if(timer.get()-start_time>=Times.conveyor_delay){
                      conveyor_speed=Speeds.conveyor_shoot_speed;
                      manual_conveyor=true;
                    }else{
                      conveyor_speed=0;
                      manual_conveyor=false;
                    }
                  }
                  if(!detected&&!prox_upper.get()){//Detects a ball as first ball
                    detected=true;
                  }else if(detected&&prox_upper.get()){//Not at proxy and has been detected, aka the first ball left
                    start_time=timer.get();
                    detected=false;
                    first=false;
                  }
                  manual_shooter = true;
              }else{
                manual_shooter=false;
                manual_conveyor=false;
                detected=false;
                first=true;
                shooter_speed=0;
                conveyor_speed=0;
                auto_state++;
              }
              break;
          }
          break;
      case WallToBallAuto:
          switch(auto_state){
            case 0:
              if(timer.get()<Times.wall_shoot_time){
                shooter_speed=Speeds.wall_shooter_speed;
                manual_shooter=true;
                if(timer.get()>Times.conveyor_first_delay){
                  conveyor_speed=Speeds.conveyor_shoot_speed;
                  manual_conveyor=true;
                }
              }else{
                conveyor_speed=0;
                shooter_speed=0;
                manual_conveyor=false;
                manual_shooter=false;
                auto_state++;
              }
              break;
            case 1:
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Distances.wall_to_ball*Distances.encoder_ratio){
                xSpeed=Speeds.auto_drive_speed;
                intake_speed=Speeds.auto_intake_speed;
              }else{
                starting_pos=dt.front_left.getSelectedSensorPosition();
                auto_state++;
              }
              break;
            case 2:
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Distances.wall_to_ball*Distances.encoder_ratio){
                xSpeed=-Speeds.auto_drive_speed;
              }else{
                stop_time=timer.get();
                auto_state++;
              }
              break;
            case 3:
              if(timer.get()-stop_time<Times.stop_dt_time){
                xSpeed=0.5;
              }else{
                xSpeed=0;
                intake_speed=0;
                start_time=timer.get();
                auto_state++;
              }
              break;
            case 4:
              if(timer.get()-start_time<Times.wall_shoot_time){
                shooter_speed=Speeds.wall_shooter_speed;
                manual_shooter=true;
                if(timer.get()-start_time>Times.conveyor_first_delay){
                  conveyor_speed=Speeds.conveyor_shoot_speed;
                  manual_conveyor=true;
                }
              }else{
                conveyor_speed=0;
                shooter_speed=0;
                manual_conveyor=false;
                manual_shooter=false;
                auto_state++;
              }
              break;
          }
        break;
      case ThreeBall:
        switch(auto_state){
          case 0://Shoot
            if(timer.get()<Times.wall_shoot_time-1){
              shooter_speed=Speeds.wall_shooter_speed;
              manual_shooter=true;
              xSpeed=0;
              if(timer.get()>Times.conveyor_first_delay){
                conveyor_speed=Speeds.conveyor_shoot_speed;
                manual_conveyor=true;
              }
            }else{
              conveyor_speed=0;
              shooter_speed=0;
              manual_conveyor=false;
              manual_shooter=false;
              auto_state++;
            }
            break;
          case 1://Drive Back
            if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Distances.wall_to_ball*Distances.encoder_ratio){
              xSpeed=Speeds.auto_drive_speed;
              intake_speed=Speeds.auto_intake_speed;
            }else{
              stop_time=timer.get();
              auto_state++;
            }
            break;
          case 2://Stop
            if(timer.get()-stop_time<Times.stop_dt_time){
              xSpeed=-0.5;
            }else{
              navx.reset();
              xSpeed=0;
              auto_state++;
            }
            break;
          case 3://turn to face next ball
            if(navx.getAngle()%360<Distances.ball_to_ball_angle-Distances.turning_error){
              zRotation=Speeds.auto_turn_speed;
            }else{
              auto_state++;
              zRotation=0;
              starting_pos=dt.front_left.getSelectedSensorPosition();
            }
            break;
          case 4://Drive to next ball
            if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Distances.bot_ball_to_ball*Distances.encoder_ratio){
              xSpeed=Speeds.auto_drive_speed;
            }else{
              auto_state++;
              stop_time=timer.get();
            }
            break;
          case 5://Stop
            if(timer.get()-stop_time<Times.stop_dt_time){
              xSpeed=-0.5;
            }else{
              xSpeed=0;
              navx.reset();
              auto_state++;
            }
            break;
          case 6://Turn to face goal
            if(navx.getAngle()%360>-Distances.ball_to_goal_angle+Distances.turning_error){
              zRotation=-Speeds.auto_turn_speed;
            }else{
              zRotation=0;
              intake_speed=0;
              auto_state++;
              starting_pos=dt.front_left.getSelectedSensorPosition();
            }
            break;
          case 7://Drive to Goal
            if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Distances.sec_ball_to_goal*Distances.encoder_ratio){
              xSpeed=-Speeds.auto_drive_speed;
              shooter_speed=Speeds.wall_shooter_speed;
            }else{
              stop_time=timer.get();
              auto_state++;
            }
            break;
          case 8://Stop
            if(timer.get()-stop_time<Times.stop_dt_time){
              xSpeed=0.5;
            }else{
              xSpeed=0;
              auto_state++;
              start_time=timer.get();
            }
            break;
          case 9://Shoot
            if(timer.get()-start_time<Times.wall_shoot_time){
              shooter_speed=Speeds.wall_shooter_speed;
                if(first){
                  conveyor_speed=Speeds.conveyor_shoot_speed;
                  manual_conveyor=true;
                }else{
                  if(timer.get()-start_time>=Times.conveyor_delay){
                    conveyor_speed=Speeds.conveyor_shoot_speed;
                    manual_conveyor=true;
                  }else{
                    conveyor_speed=0;
                    manual_conveyor=false;
                  }
                }
                if(!detected&&!prox_upper.get()){//Detects a ball as first ball
                  detected=true;
                }else if(detected&&prox_upper.get()){//Not at proxy and has been detected, aka the first ball left
                  start_time=timer.get();
                  detected=false;
                  first=false;
                }
                manual_shooter = true;
            }else{
              manual_shooter=false;
              manual_conveyor=false;
              detected=false;
              first=true;
              shooter_speed=0;
              conveyor_speed=0;
              auto_state++;
            }
            break;
        }
        break;
      case BasicAuto:
          switch(auto_state){
            case 0://Drive forward
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Distances.wall_to_ball*Distances.encoder_ratio){
                xSpeed=Speeds.auto_drive_speed;
                intake_speed=Speeds.auto_intake_speed;
              }else{
                xSpeed = 0;
                stop_time=timer.get();
                auto_state++;
              }
              break;
            case 1://Stop driving
              if(timer.get()-stop_time<Times.stop_dt_time){
                xSpeed=-0.5;
              }else{
                auto_state++;
                start_time=timer.get();
                xSpeed=0;
              }
              break;
            case 2://Intake ball
              if(timer.get()-start_time<Times.intake_ball_time){
                intake_speed=Speeds.auto_intake_speed;
              }else{
                intake_speed=0;
                starting_pos=dt.front_left.getSelectedSensorPosition();
                auto_state++;
              }
              break;
            case 3://Drive back
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Distances.ball_to_wall*Distances.encoder_ratio){
                xSpeed = -Speeds.auto_drive_speed;
              }else{
                xSpeed=0;
                start_time=timer.get();
                auto_state++;
              }
              break;
            case 4://Shoot
              if(timer.get()-start_time<Times.wall_shoot_time){
                shooter_speed=Speeds.wall_shooter_speed;
                if(timer.get()-start_time>Times.conveyor_delay){
                  conveyor_speed=Speeds.manual_conveyor_speed;
                  manual_conveyor=true;
                }
                manual_shooter=true;
              }else{
                auto_state++;
                manual_shooter=false;
                manual_conveyor=false;
                conveyor_speed=0;
                shooter_speed=0;
              }
              break;
          }
        break;
      case DriveAuto:
          switch(auto_state){
            case 0:
              if(timer.get()<2){
                xSpeed=Speeds.auto_drive_speed;
              }
              /*if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Distances.default_drive*Distances.encoder_ratio){
                xSpeed=Speeds.auto_drive_speed;
              }*/else{
                auto_state++;
                stop_time=timer.get();
                xSpeed=0;
              }
              break;
            case 1:
              if(timer.get()-stop_time<Times.stop_dt_time){
                xSpeed=-0.5;
              }else{
                auto_state++;
              }
              break;
            case 2:
              xSpeed=0;
          }
        break;
      case Shoot:
        if(timer.get()<Times.wall_shoot_time){
          shooter_speed=Speeds.wall_shooter_speed;
          manual_shooter=true;
          if(timer.get()>Times.conveyor_first_delay){
            conveyor_speed=Speeds.conveyor_shoot_speed;
            manual_conveyor=true;
          }
        }else{
          conveyor_speed=0;
          shooter_speed=0;
          manual_conveyor=false;
          manual_shooter=false;
        }
        break;
      case DefaultAuto://Start flush with wall and just shoot
          switch(auto_state){
            case 0:
              if(timer.get()<Times.wall_shoot_time){
                shooter_speed=Speeds.wall_shooter_speed;
                manual_shooter=true;
                if(timer.get()>Times.conveyor_first_delay){
                  conveyor_speed=Speeds.conveyor_shoot_speed;
                  manual_conveyor=true;
                }
              }else{
                conveyor_speed=0;
                shooter_speed=0;
                manual_conveyor=false;
                manual_shooter=false;
                auto_state++;
              }
              break;
            case 1:
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Distances.default_drive*Distances.encoder_ratio){
                xSpeed=Speeds.auto_drive_speed;
              }else{
                xSpeed = 0;
                starting_pos=dt.front_left.getSelectedSensorPosition();
                stop_time=timer.get();
                auto_state++;
              }
              break;
            case 2:
              if(timer.get()-stop_time<Times.stop_dt_time){
                xSpeed=-0.5;
              }else{
                auto_state++;
                xSpeed=0;
              }
              break;
            }
        break;
    }
    if(autoSelected == ThreeBallAuto || autoSelected == TrajectoryAuto){
      dt.set_speeds_voltage(auto_speeds.leftMetersPerSecond, auto_speeds.rightMetersPerSecond, starting_pos, start_right_pos);
    }
    else{
      dt.set_speeds(xSpeed, zRotation);//Arcade drive
    }
    //dt.set_auto_speeds(xSpeed, zRotation);//Curvature drive
    shooter.set_voltage(shooter_speed);
    conveyor.set_speed(conveyor_speed);
    intake.set_speed(intake_speed);
  }
  @Override
  public void teleopInit() {
    start_time = 0;
    stop_time=0;
    intake_speed=0;
    shooter_speed=0;
    conveyor_speed=0;
    timer.reset();
    timer.start();
    first=true;
    starting_pos = dt.front_left.getSelectedSensorPosition();
    start_right_pos = dt.front_right.getSelectedSensorPosition();
    /* factory default values */
    /*_talonL.configFactoryDefault();
    _talonR.configFactoryDefault();*/

    /* flip values so robot moves forward when stick-forward/LEDs-green */
    /*_talonL.setInverted(false); // <<<<<< Adjust this
    _talonR.setInverted(false); // <<<<<< Adjust this*/
    /*
     * WPI drivetrain classes defaultly assume left and right are opposite. call
     * this so we can apply + to both sides when moving forward. DO NOT CHANGE
     */
  }

  @Override
  public void teleopPeriodic() {
    pose = dt.odom.update(dt.getHeading(), dt.convertMeters(dt.front_left.getSelectedSensorPosition()-starting_pos), dt.convertMeters(dt.front_right.getSelectedSensorPosition()-starting_pos));
    
    xSpeed = joystick_1.getRawAxis(1) * -1; // make forward stick positive
    zRotation = joystick_1.getRawAxis(2)*0.55; // WPI Drivetrain uses positive=> right Arcade Drive
    //zRotation = joystick_1.getRawAxis(2);//Curvature Drive
    if(joystick_0.getThrottle()>0.5){
      climber_mode=true;
    }else{
      climber_mode=false;
    }

    //All automated functions run first
    //If ball detected make conveyor move to pull it
    if(prox_lower.get()){
      conveyor_speed=Speeds.index_conveyor_speed;
      conveyor_index_time=timer.get();
    }else if(!manual_conveyor&&timer.get()-conveyor_index_time>Times.index_time){//If ball is not detected, so long as it is not under manual control conveyor will stop
      conveyor_speed=0;//Greater than 0.25 seconds since detecting a ball
    }

    //If ball is detected by upper proxy stop conveyor
    if(!prox_upper.get()&&!manual_shooter){//Not upper because for some dumb reason the proxy is switched on robot
      conveyor_speed=0;
    }

    /*if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Distances.encoder_ratio*Distances.spacing&&is_spacing){
      xSpeed=Speeds.spacing_speed;
    }*/


    //All shooter controls
    if(joystick_0.getRawButtonPressed(Buttons.manual_shoot_button)){
      start_time=timer.get();
      starting_pos=dt.front_left.getSelectedSensorPosition();
      is_spacing=true;
    }

    //Manually control shooter
    /*if (joystick_0.getRawButton(Buttons.manual_shoot_button)){//Shoot charge up and back up
      shooter_speed=Speeds.shooter_volt;
      if(first){
        if(timer.get()-start_time>=Times.conveyor_first_delay){
          conveyor_speed=Speeds.conveyor_shoot_speed;
          manual_conveyor=true;
        }else{
          conveyor_speed=0;
          manual_conveyor=false;
        }
      }else{
        if(timer.get()-start_time>=Times.conveyor_delay){
          conveyor_speed=Speeds.conveyor_shoot_speed;
          manual_conveyor=true;
        }else{
          conveyor_speed=0;
          manual_conveyor=false;
        }
      }
      if(!detected&&!prox_upper.get()){//Detects a ball as first ball
        detected=true;
      }else if(detected&&prox_upper.get()){//Not at proxy and has been detected, aka the first ball left
        start_time=timer.get();
        detected=false;
        first=false;
      }
      manual_shooter = true;
    }*/

    if(joystick_0.getRawButton(Buttons.manual_shoot_button)){
      shooter_speed=Speeds.shooter_volt;
      manual_shooter=true;
    }
    /*shooter.shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    System.out.println("Mag: "+shooter.shooter.getSelectedSensorVelocity());
    shooter.shooter.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    System.out.println("Quad: "+shooter.shooter.getSelectedSensorVelocity());
    shooter.shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    System.out.println("Int: "+shooter.shooter.getSelectedSensorVelocity());*/  

    if(joystick_1.getRawButton(Buttons.manual_shoot_button)){
      shooter_back_speed=Speeds.shooter_back_volt;
      manual_shooter=true;
    }

    if (joystick_0.getRawButtonReleased(Buttons.manual_shoot_button)){
      manual_shooter = false;
      manual_conveyor=false;
      shooter_speed=0;
      conveyor_speed=0;
      start_time=0;
      detected=false;
      stop_time=0;
      first=true;
      is_spacing=false;
    }



    //Stop shooter when not manually controlled
    if (joystick_1.getRawButtonReleased(Buttons.manual_shoot_button)){
      manual_shooter = false;
      manual_conveyor=false;
      shooter_back_speed=0;
      conveyor_speed=0;
      start_time=0;
      detected=false;
      stop_time=0;
      first=true;
      is_spacing=false;
    }


    //All servo controls
    if(joystick_1.getRawButtonPressed(Buttons.intake_servo_button)){
      if(intake_servo.get()>0){
        intake_servo.set(0);
      }else{
        intake_servo.set(1);
      }
    }

    if(joystick_1.getRawButtonPressed(Buttons.shooter_servo_button)){
      if(shooter_servo_0.getPosition()>0){
        shooter_servo_0.setSpeed(-1);
        shooter_servo_1.setSpeed(-1);
      }else{
        shooter_servo_0.setSpeed(1);
        shooter_servo_1.setSpeed(1);
      }
    }


    //Intake Controls
    //Manually control intake
    if (joystick_1.getRawButton(Buttons.manual_intake_button)){
      intake_speed=Speeds.manual_intake_speed;
    }

    //Stop intake when not manually controlled
    if (joystick_1.getRawButtonReleased(Buttons.manual_intake_button)){
      intake_speed=0;
    }

    //Run intake backwards
    if (joystick_0.getRawButton(Buttons.manual_rev_intake_button)){
      intake_speed=-Speeds.manual_intake_speed;
    }

    //Stop intake backwards
    if (joystick_0.getRawButtonReleased(Buttons.manual_rev_intake_button)){
      intake_speed=0;
    }


    //Conveyor Controls
    //Manually control conveyor
    if (joystick_0.getRawButton(Buttons.manual_conveyor_button)){
      conveyor_speed = Speeds.manual_conveyor_speed;
      manual_conveyor = true;
    }

    //Stop conveyor when not manually controlled
    if (joystick_0.getRawButtonReleased(Buttons.manual_conveyor_button)){
      manual_conveyor = false;
      conveyor_speed=0;
    }

    //Manually control conveyor backwards
    if (joystick_0.getRawButton(Buttons.manual_rev_conveyor_button)){
      conveyor_speed = -Speeds.manual_conveyor_speed;
      manual_conveyor = true;
    }

    //Stop conveyor when not manually controlled backwards
    if (joystick_0.getRawButtonReleased(Buttons.manual_rev_conveyor_button)){
      manual_conveyor = false;
      conveyor_speed=0;
    }


    //Toggled buttons for climbing
    if(climber_mode){
      //Climber Controls
      //Manually control climber
      if (joystick_0.getRawButton(Buttons.manual_climber_button)){
        if(left_can_climb){
          left_climber_speed = Speeds.manual_left_climber_speed_up;
        }else{
          left_climber_speed=0;
        }
        if(right_can_climb){
          right_climber_speed = Speeds.manual_right_climber_speed_up;
        }else{
          right_climber_speed=0;
        }
        state=0;
      }

      //Stop climber when not manually controlled
      if (joystick_0.getRawButtonReleased(Buttons.manual_climber_button)){
        left_climber_speed=0;
        right_climber_speed=0;
        state=0;
      }

      //Manually control climber backwards
      if (joystick_0.getRawButton(Buttons.manual_rev_climber_button)){
        left_climber_speed = -Speeds.manual_left_climber_speed_down;
        right_climber_speed= -Speeds.manual_right_climber_speed_down;
        left_can_climb=true;
        right_can_climb=true;
        state=0;
      }

      //Stop climber when not manually controlled backwards
      if (joystick_0.getRawButtonReleased(Buttons.manual_rev_climber_button)){
        left_climber_speed=0;
        right_climber_speed=0;
        state=0;
      }

      //Manually control left climber
      if (joystick_0.getRawButton(Buttons.manual_left_climber_button)){
        if(left_can_climb){
          left_climber_speed = Speeds.manual_left_climber_speed_up;
        }else{
          left_climber_speed=0;
        }
        state=1;
      }

      //Stop left climber when not manually controlled
      if (joystick_0.getRawButtonReleased(Buttons.manual_left_climber_button)){
        left_climber_speed=0;
        state=1;
      }

      //Manually control left climber backwards
      if (joystick_0.getRawButton(Buttons.manual_rev_left_climber_button)){
        left_climber_speed = -Speeds.manual_left_climber_speed_down;
        left_can_climb=true;
        state=1;
      }

      //Stop left climber when not manually controlled backwards
      if (joystick_0.getRawButtonReleased(Buttons.manual_rev_left_climber_button)){
        left_climber_speed=0;
        state=1;
      }
      
      //Manually control right climber
      if (joystick_0.getRawButton(Buttons.manual_right_climber_button)){
        if(right_can_climb){
          right_climber_speed = Speeds.manual_right_climber_speed_up;
        }else{
          right_climber_speed=0;
        }
        state=2;
      }

      //Stop right climber when not manually controlled
      if (joystick_0.getRawButtonReleased(Buttons.manual_right_climber_button)){
        right_climber_speed=0;
        state=2;
      }

      //Manually control right climber backwards
      if (joystick_0.getRawButton(Buttons.manual_rev_right_climber_button)){
        right_climber_speed = -Speeds.manual_right_climber_speed_down;
        right_can_climb=true;
        state=2;
      }

      //Stop right climber when not manually controlled backwards
      if (joystick_0.getRawButtonReleased(Buttons.manual_rev_right_climber_button)){
        right_climber_speed=0;
        state=2;
      }
      
      //Hall effects for climbers
      if(!hall_left.get()){//Switched so if it is true and "getting" then there is nothing there
        left_can_climb=false;
      }

      if(!hall_right.get()){
        right_can_climb=false;
      }
    }
    

    //Executes all speeds
    dt.set_speeds(xSpeed, zRotation);
    
    //dt.set_auto_speeds(xSpeed, zRotation);//Curvature
    shooter.set_voltage(shooter_speed);
    shooter_back.set_voltage(shooter_back_speed);
    conveyor.set_speed(conveyor_speed);
    intake.set_speed(intake_speed);
    // System.out.println("Shooter: "+shooter.shooter.getMotorOutputVoltage());
    // System.out.println("Back: "+shooter_back.shooter.getMotorOutputVoltage());

    if(state==0){
      climber.set_speeds(left_climber_speed, right_climber_speed);
    }else if(state==1){
      climber.set_speeds(left_climber_speed,0);
    }else if(state==2){
      climber.set_speeds(0, right_climber_speed);
    }
    
    robotX.setDouble(Units.metersToFeet(pose.getX()));
    robotY.setDouble(Units.metersToFeet(pose.getY()));
    robotHeading.setDouble(pose.getRotation().getRadians());
    isFollowingPath.setBoolean(true);
    teleop_speeds = new DifferentialDriveWheelSpeeds(dt.convertMeters2(dt.front_left.getSelectedSensorVelocity()), dt.convertMeters2(dt.front_right.getSelectedSensorVelocity()));
    
    System.out.println("X: " + pose.getX() + ", Y: " + pose.getY() + " - rotation " + pose.getRotation().getDegrees());
    System.out.println("left speed: " + teleop_speeds.leftMetersPerSecond + ", right speed: " + teleop_speeds.rightMetersPerSecond);
    System.out.println("left voltage: " + dt.front_left.getMotorOutputVoltage() + " V, right voltage: " + dt.front_right.getMotorOutputVoltage());
    System.out.println("left feedforward: " + dt.getFeedforward().calculate(teleop_speeds.leftMetersPerSecond) + " V, right feedforward: " + dt.getFeedforward().calculate(teleop_speeds.rightMetersPerSecond));
    System.out.println("left PID: " + dt.getLeftPIDController().calculate(dt.front_left.getSelectedSensorPosition()-starting_pos, teleop_speeds.leftMetersPerSecond) + ", right PID: " + dt.getRightPIDController().calculate(dt.front_right.getSelectedSensorPosition()-start_right_pos, teleop_speeds.rightMetersPerSecond));
    dt.set_speeds_voltage(teleop_speeds.leftMetersPerSecond, teleop_speeds.rightMetersPerSecond, starting_pos, start_right_pos);
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.robot.Constants.*;

public class Robot extends TimedRobot {

  DriveTrain dt = new DriveTrain();
  double xSpeed = 0;
  double zRotation = 0;

  NetworkTableEntry frontSpeedNT;
  NetworkTableEntry backSpeedNT;
  NetworkTableEntry frontVoltNT;
  NetworkTableEntry backVoltNT;
  NetworkTableEntry frontVelNT;
  NetworkTableEntry backVelNT;
  NetworkTableEntry frontReadyNT;
  NetworkTableEntry backReadyNT;
  Shooter shooter = new Shooter(Ports.shooter_port);
  Shooter shooter_back = new Shooter(Ports.shooter_back_port);
  boolean manual_shooter = false;
  double shooter_speed = 0;
  double shooter_back_speed = 0;
  boolean detected = false;
  boolean idle = false;

  Conveyor conveyor = new Conveyor();
  boolean manual_conveyor = false;
  double conveyor_speed = 0;

  Intake intake = new Intake();
  double intake_speed = 0;

  Climber climber = new Climber();
  double left_climber_speed = 0;
  double right_climber_speed = 0;
  boolean left_can_climb_up = true;// Hall effects allow to climb up on left
  boolean right_can_climb_up = true;// Hall effects allow to climb up on right
  boolean left_can_climb_down = true;
  boolean right_can_climb_down=true;
  boolean left_going_up = true; //is it going up
  boolean right_going_up = true;
  boolean override_climber = false;
  boolean left_can_climb = true;
  boolean right_can_climb = true;
  int state = 0;// 0 is both, 1 is left, 2 is right
  int left_climber_pos = 0; // 0 is bottom, 1 is middle, 2 is top
  int right_climber_pos = 0;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-roboeag");
  Limelight limelight = new Limelight(table, 30);

  DigitalInput prox_lower = new DigitalInput(Ports.proxy_lower);
  DigitalInput prox_upper = new DigitalInput(Ports.proxy_upper);
  DigitalInput hall_left = new DigitalInput(Ports.hall_left);
  DigitalInput hall_right = new DigitalInput(Ports.hall_right);
  AnalogInput drive_train_speed = new AnalogInput(Ports.drive_train_speed_port);
  Servo intake_servo = new Servo(Ports.intake_servo_port);
  Spark led = new Spark(1);
  double led_state = 0;

  Joystick joystick_0 = new Joystick(Ports.joystick_0_port);
  Joystick joystick_1 = new Joystick(Ports.joystick_1_port);
  boolean climber_mode = false;

  AHRS navx = new AHRS();
  double starting_angle = 0;

  static final String DefaultAuto = "Default";
  static final String DriveAuto = "Drive";
  static final String WallToBallAuto = "WallToBallAuto";// Climbing area is "top" part
  static final String TarmacToBallAuto = "TarmacToBallAuto";
  static final String Shoot = "ShootAuto";
  static final String TurnRight = "TurnRight";
  static final String TurnLeft = "TurnLeft";
  static final String Triangle = "Triangle";
  static final String ThreeBall = "ThreeBall";
  static final String TrajectoryAuto = "TrajectoryAuto";
  static final String ThreeBallInverted = "ThreeBallInverted";
  static final String BallDetection = "BallDetection";
  SendableChooser<String> chooser = new SendableChooser<>();
  String autoSelected;
  Timer timer = new Timer();
  double starting_pos = 0;
  double start_right_pos = 0;
  int auto_state = 0;
  double auto_start_time = 0;// Only used for auto shooting
  double start_time = 0;// starting time for auto and recycled for auto shoot time

  double stop_time = 0;// stop drive time
  double conveyor_index_time = 0;

  boolean testing = true;
  double test_temp = 0;
  double test_temp_2=0;
  double test_start=0;
  int counter = 0;
  

  @Override
  public void robotInit() {
    limelight.setLEDState(1);//turn off so everyone isnt blinded
    chooser.setDefaultOption("Default Auto", DefaultAuto);
    // chooser.addOption("Ball Detect", BallDetection);
    chooser.addOption("Drive Auto", DriveAuto);
    chooser.addOption("TarmacToBallAuto", TarmacToBallAuto);
    chooser.addOption("Wall To Ball Auto", WallToBallAuto);
    chooser.addOption("Shoot Auto", Shoot);
    // chooser.addOption("Turn Right", TurnRight);
    // chooser.addOption("Turn Left", TurnLeft);
    // chooser.addOption("Triangle", Triangle);
    chooser.addOption("Three Ball", ThreeBall);
    // chooser.addOption("Three Ball Inverted", ThreeBallInverted);
    // chooser.addOption("Three Ball Auto Traj", ThreeBallAutoTraj);
    // chooser.addOption("Trajectory test", TrajectoryAuto);

    SmartDashboard.putData("Auto Choices:", chooser);
    frontSpeedNT=Shuffleboard.getTab("Shooter").add("Front Shooter Voltage",0).getEntry();
    backSpeedNT=Shuffleboard.getTab("Shooter").add("Back Shooter Voltage",0).getEntry();
    frontVoltNT=Shuffleboard.getTab("Shooter").add("FSV",0).getEntry();
    backVoltNT=Shuffleboard.getTab("Shooter").add("BSV",0).getEntry();
    frontVelNT=Shuffleboard.getTab("Shooter").add("Front Shooter Velocity", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
    backVelNT=Shuffleboard.getTab("Shooter").add("Back Shooter Velocity", 0).getEntry();
    frontReadyNT=Shuffleboard.getTab("Shooter").add("Front Ready", false).getEntry();
    backReadyNT=Shuffleboard.getTab("Shooter").add("Back Ready", false).getEntry();
    // CameraServer.startAutomaticCapture();
    //CameraServer.startAutomaticCapture();//Call twice to automatically create both cameras and have them as optional displays
    limelight.setLEDState(3);
  }

  @Override
  public void autonomousInit(){
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
    shooter_back_speed=0;
    xSpeed=0;
    zRotation=0;
    stop_time=0;
    navx.reset();
    dt.coast();
  }
  @Override
  public void autonomousPeriodic(){
    //If ball detected make conveyor move to pull it
    if(prox_lower.get()){
      conveyor_speed=Speeds.index_conveyor_speed;
      conveyor_index_time=timer.get();
    }else if(!manual_conveyor&&timer.get()-conveyor_index_time>0.25){//If ball is not detected, conveyor will stop
      conveyor_speed=0;
    }

    // If ball is detected by upper proxy stop conveyor
    if (!prox_upper.get() && !manual_shooter) {
      conveyor_speed = 0;
    }

    // System.out.println(xSpeed);
    // System.out.println(navx.getAngle());
    //Determine what auto to run
    switch(autoSelected){

      case BallDetection:
        int val = drive_train_speed.getValue();
        System.out.println(val);
        break;

      case TurnRight:
        if (navx.getAngle() % 360 < 45 - Distances.turning_error) {
          zRotation = Speeds.auto_turn_speed;
        } else {
          zRotation = 0;
        }
        break;
      case TurnLeft:
        if (navx.getAngle() % 360 > -45 + Distances.turning_error) {
          zRotation = -Speeds.auto_turn_speed;
        } else {
          zRotation = 0;
        }
        break;
      case Triangle:
        switch (auto_state) {
          case 0:
            if (Math.abs(dt.front_left.getSelectedSensorPosition() - starting_pos) < 96 * Distances.encoder_ratio) {
              xSpeed = 0.3;
            } else {
              xSpeed = 0;
              navx.reset();
              auto_state++;
            }
            break;
          case 1:
            if (navx.getAngle() % 360 < 90 - Distances.turning_error) {
              zRotation = Speeds.auto_turn_speed;
            } else {
              zRotation = 0;
              auto_state++;
              starting_pos = dt.front_left.getSelectedSensorPosition();
            }
            break;
          case 2:
            if (Math.abs(dt.front_left.getSelectedSensorPosition() - starting_pos) < 72 * Distances.encoder_ratio) {
              xSpeed = 0.3;
            } else {
              xSpeed = 0;
              auto_state++;
              navx.reset();
            }
            break;
          case 3:
            if (navx.getAngle() % 360 < 90 - Distances.turning_error) {
              zRotation = Speeds.auto_turn_speed;
            } else {
              zRotation = 0;
              auto_state++;
              navx.reset();
              starting_pos = dt.front_left.getSelectedSensorPosition();
            }
            break;
          case 4:
            if (navx.getAngle() % 360 < 60 - Distances.turning_error) {
              zRotation = Speeds.auto_turn_speed;
            } else {
              zRotation = 0;
            }
            if (Math.abs(dt.front_left.getSelectedSensorPosition() - starting_pos) < 120 * Distances.encoder_ratio) {
              xSpeed = 0.3;
            } else {
              xSpeed = 0;
            }
            break;
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
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Distances.ball_to_line*Distances.encoder_ratio){
                xSpeed=-Speeds.auto_drive_speed;
                shooter_speed=Speeds.idle_shooter_speed;
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
              shooter_speed=Speeds.shooter_volt_far;
              shooter_back_speed=Speeds.shooter_back_volt_far;
              if(shooter.is_ready(shooter.get_vel_threshold(true))&&//If front is at desired velocity
              shooter_back.is_ready()){
                conveyor_speed=Speeds.conveyor_shoot_speed;
                manual_conveyor=true;
              }
                manual_shooter = true;
              break;
          }
          break;
        case WallToBallAuto:
          switch (auto_state) {
            case 0:
              if (timer.get() < Times.wall_shoot_time) {
                shooter_speed = Speeds.shooter_volt_close;
                shooter_back_speed = Speeds.shooter_back_volt_close;;
                manual_shooter = true;
                if (shooter.is_ready(shooter.get_vel_threshold(true))&&//If front is at desired velocity
                shooter_back.is_ready()){
                  conveyor_speed = Speeds.conveyor_shoot_speed;
                  manual_conveyor = true;
                }
              } else {
                conveyor_speed = 0;
                shooter_back_speed=0;
                manual_conveyor = false;
                manual_shooter = false;
                auto_state++;
              }
              break;
            case 1:
              if (Math.abs(dt.front_left.getSelectedSensorPosition() - starting_pos) < Distances.wall_to_ball
                  * Distances.encoder_ratio) {
                xSpeed = Speeds.auto_drive_speed;
                intake_speed = Speeds.auto_intake_speed;
              } else {
                starting_pos = dt.front_left.getSelectedSensorPosition();
                auto_state++;
              }
              break;
            case 2:
              if (Math.abs(dt.front_left.getSelectedSensorPosition() - starting_pos) < Distances.wall_to_ball
                  * Distances.encoder_ratio) {
                xSpeed = -Speeds.auto_drive_speed;
              } else {
                stop_time = timer.get();
                auto_state++;
              }
              break;
            case 3:
              if (timer.get() - stop_time < Times.stop_dt_time) {
                xSpeed = 0.5;
              } else {
                xSpeed = 0;
                intake_speed = 0;
                start_time = timer.get();
                auto_state++;
              }
              break;
            case 4:
              shooter_speed=Speeds.shooter_volt_close;
              shooter_back_speed=Speeds.shooter_back_volt_close;
              if(shooter.is_ready(shooter.get_vel_threshold(true))&&//If front is at desired velocity
              shooter_back.is_ready()){
                conveyor_speed=Speeds.conveyor_shoot_speed;
                manual_conveyor=true;
              }
                manual_shooter = true;
              break;
          }
          break;
      case ThreeBall:
        switch(auto_state){
          case 0://Shoot
            if(timer.get()<Times.wall_shoot_time-1){//If no encoder remove the -1
              shooter_speed=Speeds.shooter_volt_close;
              shooter_back_speed=Speeds.shooter_back_volt_close;
              manual_shooter=true;
              xSpeed=0;
              //if(shooter.is_ready(shooter.get_vel_threshold(true))&&//If front is at desired velocity
              //shooter_back.is_ready()){
              if(timer.get()>1){//If encoder is broken, us this and switch elsewhere
                conveyor_speed=Speeds.conveyor_shoot_speed;
                manual_conveyor=true;
              }
            }else{
              conveyor_speed=0;
              shooter_back_speed=0;
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
              shooter_speed=Speeds.idle_shooter_speed;
              shooter_back_speed=Speeds.shooter_back_volt_close;
            }else{
              stop_time=timer.get();
              auto_state++;
            }
            break;
          case 8://Stop
            if(timer.get()-stop_time<Times.stop_dt_time){
              xSpeed=0.4;
            }else{
              xSpeed=0;
              auto_state++;
              start_time=timer.get();
            }
            break;
          case 9://Shoot
            shooter_speed=Speeds.shooter_volt_far;
            shooter_back_speed=Speeds.shooter_back_volt_far;
            //if(shooter.is_ready(shooter.get_vel_threshold(true))&&//If front is at desired velocity
            //shooter_back.is_ready(shooter.get_vel_threshold(false))){
            if(timer.get()-start_time>2){
              conveyor_speed=Speeds.conveyor_shoot_speed;
              manual_conveyor=true;
            }
              manual_shooter = true;
            break;
        }
        break;
      case ThreeBallInverted:
        switch(auto_state){
          case 0://Shoot
            if(timer.get()<Times.wall_shoot_time-1){
              shooter_speed=Speeds.shooter_volt_close;
              shooter_back_speed=Speeds.shooter_back_volt_close;
              manual_shooter=true;
              xSpeed=0;
              if(shooter.is_ready(shooter.get_vel_threshold(true))&&//If front is at desired velocity
              shooter_back.is_ready()){
                conveyor_speed=Speeds.conveyor_shoot_speed;
                manual_conveyor=true;
              }
            }else{
              conveyor_speed=0;
              shooter_speed=0;
              shooter_back_speed=0;
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

        }
        break;
      case DriveAuto:
        switch(auto_state){
            case 0:
              /*if(timer.get()<2){
                xSpeed=Speeds.auto_drive_speed;
              }*/
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Distances.default_drive*Distances.encoder_ratio){
                xSpeed=Speeds.auto_drive_speed;
              }else{
                auto_state++;
                stop_time=timer.get();
                xSpeed=0;
              }
              break;
            case 1:
              if(timer.get()-stop_time<Times.stop_dt_time){
                xSpeed=-Speeds.auto_drive_speed;
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
          shooter_speed=Speeds.shooter_volt_close;
          shooter_back_speed=Speeds.shooter_back_volt_close;
          manual_shooter=true;
          if(shooter.is_ready(shooter.get_vel_threshold(true))&&//If front is at desired velocity
          shooter_back.is_ready()){
          //if(timer.get()>2){
            conveyor_speed=Speeds.conveyor_shoot_speed;
            manual_conveyor=true;
          }
        }else{
          conveyor_speed=0;
          shooter_speed=0;
          shooter_back_speed=0;
          manual_conveyor=false;
          manual_shooter=false;
        }
        break;
      case DefaultAuto://Start flush with wall and just shoot
        switch(auto_state){
            case 0:
              if(timer.get()<Times.wall_shoot_time){
                shooter_speed=Speeds.shooter_volt_close;
                shooter_back_speed=Speeds.shooter_back_volt_close;
                manual_shooter=true;
                if(shooter.is_ready(shooter.get_vel_threshold(true))&&//If front is at desired velocity
                shooter_back.is_ready()){
                //if(timer.get()>2){//Uncomment if encoder is broken and comment out above if statement
                  conveyor_speed=Speeds.conveyor_shoot_speed;
                  manual_conveyor=true;
                }
              }else{
                conveyor_speed=0;
                shooter_speed=0;
                shooter_back_speed=0;
                manual_conveyor=false;
                manual_shooter=false;
                auto_state++;
              }
              break;
            case 1:
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Distances.default_drive*Distances.encoder_ratio){
                xSpeed=Speeds.auto_drive_speed;
              }else{
                auto_state++;
                stop_time=timer.get();
                xSpeed=0;
              }
              break;
            case 2:
              if(timer.get()-stop_time<Times.stop_dt_time){
                xSpeed=-Speeds.auto_drive_speed;
              }else{
                xSpeed=0;
                auto_state++;
              }
              break;
        }
        break;
    }

    dt.set_speeds(xSpeed, zRotation);//Arcade drive
    //dt.set_auto_speeds(xSpeed, zRotation);//Curvature drive
    shooter.set_voltage(shooter_speed);
    shooter_back.set_voltage(shooter_back_speed);
    conveyor.set_speed(conveyor_speed);
    intake.set_speed(intake_speed);
  }

  @Override
  public void teleopInit() {
    //shooterSpeed = Shuffleboard.getTab("Shooter").add("Front Left Motor Speed",Speeds.auto_drive_speed).getEntry();
    limelight.setLEDState(3);
    start_time = 0;
    stop_time=0;
    intake_speed=0;
    shooter_speed=0;
    shooter_back_speed=0;
    conveyor_speed=0;
    left_climber_pos = 0;
    right_climber_pos = 0;
    idle=false;
    timer.reset();
    timer.start();
    left_going_up = true;
    right_going_up = true;
    //dt.coast();
    dt.brake();
    climber.reset_pos();
    manual_shooter=false;
    manual_conveyor=false;
  }

  @Override
  public void teleopPeriodic() {
    frontVoltNT.setDouble(shooter.shooter.getMotorOutputVoltage());
    // backVoltNT.setDouble(shooter_back.shooter.getMotorOutputVoltage());
    xSpeed = joystick_1.getRawAxis(1) * -1; // make forward stick positive
    zRotation = joystick_1.getRawAxis(2)*0.55; // WPI Drivetrain uses positive=> right Arcade Drive
    //zRotation = joystick_1.getRawAxis(2);//Curvature Drive

    if (joystick_0.getThrottle() > 0.5) {
      climber_mode = true;
    } else {
      climber_mode = false;
    }

    // All automated functions run first
    // If ball detected make conveyor move to pull it
    if (prox_lower.get()) {
      conveyor_speed = Speeds.index_conveyor_speed;
      conveyor_index_time = timer.get();
    } else if (!manual_conveyor && timer.get() - conveyor_index_time > Times.index_time) {// If ball is not detected, so long as it is not under manual control conveyor will stop
      conveyor_speed = 0;// Greater than 0.25 seconds since detecting a ball
    }

    // If ball is detected by upper proxy stop conveyor
    if (!prox_upper.get() && !manual_shooter) {// Not upper because for some dumb reason the proxy is switched on robot
      conveyor_speed = 0;
      led_state=0.59;
    }else{
      led_state=0.75;
    }

    if(!climber_mode){
      
      if(idle){
        shooter_speed=Speeds.idle_shooter_speed;
      }else{
        shooter_speed=0;
      }
      //Manually control shooter
      //Shoot Far
      if(joystick_0.getRawButton(Buttons.shoot_button_far)){
        shooter_speed=Speeds.shooter_volt_far;
        shooter_back_speed=Speeds.shooter_back_volt_far;
        frontReadyNT.setBoolean(shooter.is_ready(shooter.get_vel_threshold(true)));
        backReadyNT.setBoolean(shooter_back.is_ready(shooter_back.get_vel_threshold(false)));
        if(shooter.is_ready(shooter.get_vel_threshold(true))&&//If front is at desired velocity
        shooter_back.is_ready(shooter_back.get_vel_threshold(false))){//If back is at desired velocity
          conveyor_speed=Speeds.conveyor_shoot_speed;
          manual_conveyor=true;
        }else{
          conveyor_speed=0;
          manual_conveyor=false;
        }
        manual_shooter=true;
      }
      
      if(joystick_0.getRawButton(Buttons.shoot_button_close)){
        shooter_speed=Speeds.shooter_volt_close;
        shooter_back_speed=Speeds.shooter_back_volt_close;
        frontReadyNT.setBoolean(shooter.is_ready(shooter.get_vel_threshold(true)));
        backReadyNT.setBoolean(shooter_back.is_ready());
        if(shooter.is_ready(shooter.get_vel_threshold(true))&&//If front is at desired velocity
          shooter_back.is_ready()){//If back is at desired velocity
          conveyor_speed=Speeds.conveyor_shoot_speed;
          manual_conveyor=true;
        }else{
          conveyor_speed=0;
          manual_conveyor=false;
        }
        manual_shooter=true;
      }

      // Stop shooter when not manually controlled
      if (joystick_0.getRawButtonReleased(Buttons.shoot_button_far)) {
        frontReadyNT.setBoolean(shooter.reset_is_ready());
        backReadyNT.setBoolean(shooter_back.reset_is_ready());
        manual_shooter = false;
        manual_conveyor = false;
        shooter_speed = 0;
        shooter_back_speed=0;
        conveyor_speed = 0;
      }

      //Stop shooter when not manually controlled
      if (joystick_0.getRawButtonReleased(Buttons.shoot_button_close)){
        frontReadyNT.setBoolean(shooter.reset_is_ready());
        backReadyNT.setBoolean(shooter_back.reset_is_ready());
        manual_shooter = false;
        manual_conveyor=false;
        shooter_speed=0;
        shooter_back_speed=0;
        conveyor_speed=0;
      }
      
      if(joystick_0.getRawButtonPressed(Buttons.idle_shoot_button)){
        idle=!idle;
      }

      // Manual Limelight control
      if (joystick_0.getRawButton(Buttons.limelight_button)) {
        zRotation=dt.get_raw_speeds(limelight.getAlignmentAdjustment());
        limelight.setLEDState(3);
      }

      if (joystick_0.getRawButtonReleased(Buttons.limelight_button)) {
        zRotation=0;
      }
    }
    
    if(joystick_0.getRawButton(Buttons.manual_shoot_button)){
      shooter_speed=Speeds.shooter_volt_close;
      shooter_back_speed=Speeds.shooter_back_volt_close;
      manual_shooter=true;
    }
    if(joystick_0.getRawButtonReleased(Buttons.manual_shoot_button)){
      shooter_speed=0;
      shooter_back_speed=0;
      manual_shooter=false;
    }


    //All servo controls
    if(joystick_1.getRawButtonPressed(Buttons.intake_servo_button)){
      if(intake_servo.get()>0){
        intake_servo.set(0);
      } else {
        intake_servo.set(1);
      }
    }

    // Intake Controls
    // Manually control intake
    if (joystick_1.getRawButton(Buttons.manual_intake_button)) {
      intake_speed = Speeds.manual_intake_speed;
    }

    // Stop intake when not manually controlled
    if (joystick_1.getRawButtonReleased(Buttons.manual_intake_button)) {
      intake_speed = 0;
    }

    // Run intake backwards
    if (joystick_0.getRawButton(Buttons.manual_rev_intake_button)) {
      intake_speed = -Speeds.manual_intake_speed;
    }

    // Stop intake backwards
    if (joystick_0.getRawButtonReleased(Buttons.manual_rev_intake_button)) {
      intake_speed = 0;
    }

    // Conveyor Controls
    // Manually control conveyor
    if (joystick_0.getRawButton(Buttons.manual_conveyor_button)) {
      conveyor_speed = Speeds.manual_conveyor_speed;
      manual_conveyor = true;
    }

    // Stop conveyor when not manually controlled
    if (joystick_0.getRawButtonReleased(Buttons.manual_conveyor_button)) {
      manual_conveyor = false;
      conveyor_speed = 0;
    }

    // Manually control conveyor backwards
    if (joystick_0.getRawButton(Buttons.manual_rev_conveyor_button)) {
      conveyor_speed = -Speeds.manual_conveyor_speed;
      manual_conveyor = true;
    }

    // Stop conveyor when not manually controlled backwards
    if (joystick_0.getRawButtonReleased(Buttons.manual_rev_conveyor_button)) {
      manual_conveyor = false;
      conveyor_speed = 0;
    }
    // up == true, middle == true, then can go up or down
    // up == false, middle == true, then can go up or down
    // up == true, middle == false, 

    //state 0: can only move up
    // state 1

    if(joystick_0.getRawButtonPressed(Buttons.climber_override)){
      override_climber=!override_climber;
    }
    // Toggled buttons for climbing
    if (climber_mode) {
      led_state=-0.99;
      if(!override_climber){
        if (joystick_0.getRawButton(Buttons.manual_climber_button)) {
          if (climber.get_left_pos()<Distances.mid_climb) {
            left_climber_speed = Speeds.manual_left_climber_speed_up;
          } else {
            left_climber_speed = 0;
          }
          if (climber.get_right_pos()<Distances.mid_climb) {
            right_climber_speed = Speeds.manual_right_climber_speed_up;
          } else {
            right_climber_speed = 0;
          }
          state = 0;
        }
  
        // Stop climber when not manually controlled
        if (joystick_0.getRawButtonReleased(Buttons.manual_climber_button)) {
          left_climber_speed = 0;
          right_climber_speed = 0;
          state = 0;
        }

        if (joystick_0.getRawButton(Buttons.manual_rev_climber_button)) {
          if(climber.get_left_pos()>Distances.down_dist){
            left_climber_speed = -Speeds.manual_left_climber_speed_down;
          }else{
            left_climber_speed=0;
          }
          if(climber.get_right_pos()>Distances.down_dist){
            right_climber_speed = -Speeds.manual_right_climber_speed_down;
          }else{
            right_climber_speed=0;
          }
          state = 0;
        }
  
        // Stop climber when not manually controlled backwards
        if (joystick_0.getRawButtonReleased(Buttons.manual_rev_climber_button)) {
          left_climber_speed = 0;
          right_climber_speed = 0;
          state = 0;
        }

      }else{
        if (joystick_0.getRawButton(Buttons.manual_climber_button)) {
          if (left_can_climb) {
            left_climber_speed = Speeds.manual_left_climber_speed_up;
          } else {
            left_climber_speed = 0;
          }
          if (right_can_climb) {
            right_climber_speed = Speeds.manual_right_climber_speed_up;
          } else {
            right_climber_speed = 0;
          }
          state = 0;
        }
  
        // Stop climber when not manually controlled
        if (joystick_0.getRawButtonReleased(Buttons.manual_climber_button)) {
          left_climber_speed = 0;
          right_climber_speed = 0;
          state = 0;
        }
  
        // Manually control climber backwards
        if (joystick_0.getRawButton(Buttons.manual_rev_climber_button)) {
          left_climber_speed = -Speeds.manual_left_climber_speed_down;
          right_climber_speed = -Speeds.manual_right_climber_speed_down;
          left_can_climb = true;
          right_can_climb = true;
          state = 0;
        }
  
        // Stop climber when not manually controlled backwards
        if (joystick_0.getRawButtonReleased(Buttons.manual_rev_climber_button)) {
          left_climber_speed = 0;
          right_climber_speed = 0;
          state = 0;
        }
        
        // Manually control left climber
        if (joystick_0.getRawButton(Buttons.manual_left_climber_button)) {
          if (left_can_climb) {
            left_climber_speed = Speeds.manual_left_climber_speed_up;
          } else {
            left_climber_speed = 0;
          }
          state = 1;
        }
  
        // Stop left climber when not manually controlled
        if (joystick_0.getRawButtonReleased(Buttons.manual_left_climber_button)) {
          left_climber_speed = 0;
          state = 1;
        }
        
        // Manually control left climber backwards
        if (joystick_0.getRawButton(Buttons.manual_rev_left_climber_button)) {
          left_climber_speed = -Speeds.manual_left_climber_speed_down;
          left_can_climb = true;
          state = 1;
        }

        // Stop left climber when not manually controlled backwards
        if (joystick_0.getRawButtonReleased(Buttons.manual_rev_left_climber_button)) {
          left_climber_speed = 0;
          state = 1;
        }
  
        // Manually control right climber
        if (joystick_0.getRawButton(Buttons.manual_right_climber_button)) {
          if (right_can_climb) {
            right_climber_speed = Speeds.manual_right_climber_speed_up;
          } else {
            right_climber_speed = 0;
          }
          state = 2;
        }
  
        // Stop right climber when not manually controlled
        if (joystick_0.getRawButtonReleased(Buttons.manual_right_climber_button)) {
          right_climber_speed = 0;
          state = 2;
        }
  
        // Manually control right climber backwards
        if (joystick_0.getRawButton(Buttons.manual_rev_right_climber_button)) {
          right_climber_speed = -Speeds.manual_right_climber_speed_down;
          right_can_climb = true;
          state = 2;
        }
  
        // Stop right climber when not manually controlled backwards
        if (joystick_0.getRawButtonReleased(Buttons.manual_rev_right_climber_button)) {
          right_climber_speed = 0;
          state = 2;
        }
  
        // Hall effects for climbers
        if (!hall_left.get()) {// Switched so if it is true and "getting" then there is nothing there
          left_can_climb = false;
        }
  
        if (!hall_right.get()) {
          right_can_climb = false;
        }
      }
    }
    
    // Executes all speeds
    dt.set_speeds(xSpeed, zRotation);
    shooter.set_voltage(shooter_speed);
    shooter_back.set_voltage(shooter_back_speed);
    conveyor.set_speed(conveyor_speed);
    intake.set_speed(intake_speed);
    led.set(led_state);

    if (state == 0) {
      climber.set_speeds(left_climber_speed, right_climber_speed);
    } else if (state == 1) {
      climber.set_speeds(left_climber_speed, 0);
    } else if (state == 2) {
      climber.set_speeds(0, right_climber_speed);
    }
    // System.out.println("Left Pos: "+climber.get_left_pos());
    // System.out.println("Right Pos:"+ climber.get_right_pos());
    // System.out.println("Climber mode: "+ climber_mode);
    // System.out.println("Left Speed: "+left_climber_speed);
    System.out.println("Override: "+override_climber);
    /*
    String state_name = "";
    if(left_climber_pos == 0){
      state_name = "bottom";
    }
    else if(left_climber_pos == 1){
      state_name = "middle";
    }
    else{
      state_name = "top";
    }
    System.out.println("state: " + state_name);*/
  }

  @Override
  public void testInit(){
    limelight.setLEDState(3);
    start_time = 0;
    stop_time=0;
    intake_speed=0;
    shooter_speed=0;
    shooter_back_speed=0;
    conveyor_speed=0;
    left_climber_pos = 0;
    right_climber_pos = 0;
    idle=false;
    timer.reset();
    timer.start();
    left_going_up = true;
    right_going_up = true;
    dt.coast();
    climber.reset_pos();
    manual_shooter=false;
    manual_conveyor=false;
  }

  @Override
  public void testPeriodic(){
    // SmartDashboard.putNumber("Shooter/Front Shooter Velocity", shooter.get_velocity());
    // SmartDashboard.putNumber("Shooter/Back Shooter Velocity", shooter_back.get_velocity());
    frontVelNT.setDouble(shooter.get_velocity());
    backVelNT.setDouble(shooter_back.get_velocity());
    frontVoltNT.setDouble(shooter.shooter.getMotorOutputVoltage());
    backVoltNT.setDouble(shooter_back.shooter.getMotorOutputVoltage());

    if(joystick_0.getRawButton(Buttons.shoot_button_far)){
      shooter_speed=frontSpeedNT.getDouble(0);
      shooter_back_speed=backSpeedNT.getDouble(0);
      frontReadyNT.setBoolean(shooter.is_ready(shooter.get_vel_threshold(true)));
      backReadyNT.setBoolean(shooter_back.is_ready(shooter_back.get_vel_threshold(false)));
      if(shooter.is_ready(shooter.get_vel_threshold(true))&&//If front is at desired velocity
      shooter_back.is_ready(shooter_back.get_vel_threshold(false))){//If back is at desired velocity
        conveyor_speed=Speeds.conveyor_shoot_speed;
        manual_conveyor=true;
      }else{
        conveyor_speed=0;
        manual_conveyor=false;
      }
      manual_shooter=true;
      led.set(0.59);
    }

    if (joystick_0.getRawButtonReleased(Buttons.shoot_button_far)) {
      frontReadyNT.setBoolean(shooter.reset_is_ready());
      backReadyNT.setBoolean(shooter_back.reset_is_ready());
      manual_shooter = false;
      manual_conveyor = false;
      shooter_speed = 0;
      shooter_back_speed=0;
      conveyor_speed = 0;
      led.set(0.73);
    }
  }
}
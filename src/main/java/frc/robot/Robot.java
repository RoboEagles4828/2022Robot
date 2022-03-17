// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.CounterBase;

public class Robot extends TimedRobot {

  DriveTrain dt = new DriveTrain();
  double xSpeed = 0;
  double zRotation = 0;
  
  Shooter shooter = new Shooter();
  boolean manual_shooter = false;
  double shooter_speed = 0;
  boolean detected = false;

  Conveyor conveyor = new Conveyor();
  boolean manual_conveyor = false;
  double conveyor_speed = 0;

  Intake intake = new Intake();
  double intake_speed = 0;

  Climber climber = new Climber();
  double climber_speed = 0;
  double left_climber_speed = 0;
  double right_climber_speed = 0;
  int state = 0;//0 is both, 1 is left, 2 is right

  DigitalInput prox_lower = new DigitalInput(Constants.proxy_lower);
  DigitalInput prox_upper = new DigitalInput(Constants.proxy_upper);

  Servo intake_servo = new Servo(Constants.servo_port);

  Joystick joystick_0 = new Joystick(Constants.joystick_0_port);
  Joystick joystick_1 = new Joystick(Constants.joystick_1_port);

  AHRS navx = new AHRS();

  static final String DefaultAuto = "Default";
  static final String BasicAuto = "DriveShoot";
  static final String DriveAuto = "Drive";
  SendableChooser<String> chooser = new SendableChooser<>();
  String autoSelected;
  Timer timer = new Timer();
  double starting_pos = 0;
  int auto_state = 0;
  double start_time = 0;//starting time for auto and recycled for auto shoot time

  double stop_time = 0;//stop drive time
  double conveyor_index_time = 0;

  @Override
  public void robotInit(){
    chooser.setDefaultOption("Default Auto", DefaultAuto);
    chooser.addOption("Basic Auto", BasicAuto);
    chooser.addOption("Drive Auto", DriveAuto);
    SmartDashboard.putData("Auto Choices:", chooser);
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();//Call twice to automatically create both cameras and have them as optional displays
    //dt.front_left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);//Defaults to integrated sensor, this is quadrature
    dt.front_left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    //navx.calibrate();
  }

  @Override
  public void autonomousInit(){
    autoSelected=chooser.getSelected();
    System.out.println("Auto selected: " + autoSelected);
    timer.reset();
    timer.start();
    starting_pos=dt.front_left.getSelectedSensorPosition();
    intake_servo.set(1);
    auto_state=0;
    conveyor_speed=0;
    intake_speed=0;
    shooter_speed=0;
    xSpeed=0;
    zRotation=0;
    stop_time=0;
  }

  @Override
  public void autonomousPeriodic(){
    //If ball detected make conveyor move to pull it
    if(prox_lower.get()){
      conveyor_speed=Constants.index_conveyor_speed;
      conveyor_index_time=timer.get();
    }else if(!manual_conveyor&&timer.get()-conveyor_index_time>0.25){//If ball is not detected, conveyor will stop
      conveyor_speed=0;
    }

    //If ball is detected by upper proxy stop conveyor
    if(prox_upper.get()&&!manual_shooter){
      conveyor_speed=0;
    }

    //Determine what auto to run
    switch(autoSelected){
      case BasicAuto:
          switch(auto_state){
            case 0:
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Constants.wall_to_ball*Constants.encoder_ratio){
                xSpeed=Constants.auto_drive_speed;
                intake_speed=Constants.auto_intake_speed;
              }else{
                xSpeed = 0;
                start_time=timer.get();
                auto_state++;
              }
              break;
            case 1:
              if(timer.get()-start_time<Constants.intake_ball_time){
                intake_speed=Constants.auto_intake_speed;
              }else{
                intake_speed=0;
                starting_pos=dt.front_left.getSelectedSensorPosition();
                auto_state++;
              }
              break;
            case 2:
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Constants.ball_to_wall*Constants.encoder_ratio){
                xSpeed = -Constants.auto_drive_speed;
              }else{
                xSpeed=0;
                start_time=timer.get();
                auto_state++;
              }
              break;
            case 3:
              if(timer.get()-start_time<Constants.wall_shoot_time){
                shooter_speed=Constants.wall_shooter_speed;
                if(timer.get()-start_time>Constants.conveyor_delay){
                  conveyor_speed=Constants.manual_conveyor_speed;
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
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<360*Constants.encoder_ratio){
                xSpeed=Constants.auto_drive_speed;
              }else{
                auto_state++;
                stop_time=timer.get();
                xSpeed=0;
              }
              break;
            case 1:
              if(timer.get()-stop_time<Constants.stop_dt_time){
                xSpeed=-0.5;
              }else{
                auto_state++;
              }
              break;
            case 2:
              xSpeed=0;
          }
        break;
      case DefaultAuto://Start flush with wall and just shoot
        if(timer.get()<Constants.wall_shoot_time){
          shooter_speed=Constants.wall_shooter_speed;
          manual_shooter=true;
          if(timer.get()>Constants.conveyor_delay){
            conveyor_speed=Constants.conveyor_shoot_speed;
            manual_conveyor=true;
          }
        }else{
          conveyor_speed=0;
          manual_conveyor=false;
          manual_shooter=false;
        }
        break;
    }

    dt.set_speeds(xSpeed, zRotation);
    shooter.set_speed(shooter_speed);
    conveyor.set_speed(conveyor_speed);
    intake.set_speed(intake_speed);
  }
  @Override
  public void teleopInit() {
    starting_pos=dt.front_left.getSelectedSensorPosition();
    start_time = 0;
    intake_speed=0;
    shooter_speed=0;
    conveyor_speed=0;
    timer.reset();
    timer.start();
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
    xSpeed = joystick_1.getRawAxis(1) * -1; // make forward stick positive
    zRotation = joystick_1.getRawAxis(2)*0.4; // WPI Drivetrain uses positive=> right
    //If ball detected make conveyor move to pull it
    if(prox_lower.get()){
      conveyor_speed=Constants.index_conveyor_speed;
      conveyor_index_time=timer.get();
    }else if(!manual_conveyor&&timer.get()-conveyor_index_time>0.3){//If ball is not detected, so long as it is not under manual control conveyor will stop
      conveyor_speed=0;//Greater than 0.25 seconds since detecting a ball
    }

    //If ball is detected by upper proxy stop conveyor
    if(!prox_upper.get()&&!manual_shooter){//Not upper because for some dumb reason the proxy is switched on robot
      conveyor_speed=0;
    }

    //Manually control shooter
    if (joystick_0.getRawButton(Constants.manual_shoot_button)){
      shooter_speed=Constants.manual_shooter_speed;
      manual_shooter = true;
    }

    //Stop shooter when not manually controlled
    if (joystick_0.getRawButtonReleased(Constants.manual_shoot_button)){
      manual_shooter = false;
      shooter_speed=0;
    }

    if(joystick_1.getRawButtonPressed(Constants.manual_shoot_button)){
      start_time=timer.get();
    }

    //Manually control shooter
    if (joystick_1.getRawButton(Constants.manual_shoot_button)){
      shooter_speed=Constants.manual_shooter_speed;
      if(timer.get()-start_time>=1){
        conveyor_speed=Constants.conveyor_shoot_speed;
        manual_conveyor=true;
      }else{
        conveyor_speed=0;
        manual_conveyor=false;
      }
      /*if(!detected&&prox_upper.get()){
        detected=true;
      }else if(!prox_upper.get()&&detected){
        start_time=timer.get();
      }*/
      manual_shooter = true;
    }

    //Stop shooter when not manually controlled
    if (joystick_1.getRawButtonReleased(Constants.manual_shoot_button)){
      manual_shooter = false;
      manual_conveyor=false;
      shooter_speed=0;
      conveyor_speed=0;
      start_time=0;
      detected=false;
    }

    if(joystick_1.getRawButtonPressed(Constants.servo_switch_button)){
      if(intake_servo.get()>0){
        intake_servo.set(0);
      }else{
        intake_servo.set(1);
      }
    }
    
    //Manually control intake
    if (joystick_0.getRawButton(Constants.manual_intake_button)){
      intake_speed=Constants.manual_intake_speed;
    }

    //Stop intake when not manually controlled
    if (joystick_0.getRawButtonReleased(Constants.manual_intake_button)){
      intake_speed=0;
    }

    //Manually control intake
    if (joystick_1.getRawButton(Constants.manual_intake_button_1)){
      intake_speed=Constants.manual_intake_speed;
    }

    //Stop intake when not manually controlled
    if (joystick_1.getRawButtonReleased(Constants.manual_intake_button_1)){
      intake_speed=0;
    }

    //Manually control intake for other joystick
    if (joystick_1.getRawButton(Constants.manual_intake_button)){
      intake_speed=Constants.manual_intake_speed;
    }

    //Stop intake when not manually controlled other joystick
    if (joystick_1.getRawButtonReleased(Constants.manual_intake_button)){
      intake_speed=0;
    }

    //Run intake backwards
    if (joystick_0.getRawButton(Constants.manual_rev_intake_button)){
      intake_speed=-Constants.manual_intake_speed;
    }

    //Stop intake backwards
    if (joystick_0.getRawButtonReleased(Constants.manual_rev_intake_button)){
      intake_speed=0;
    }

    //Manually control conveyor
    if (joystick_0.getRawButton(Constants.manual_conveyor_button)){
      conveyor_speed = Constants.manual_conveyor_speed;
      manual_conveyor = true;
    }

    //Stop conveyor when not manually controlled
    if (joystick_0.getRawButtonReleased(Constants.manual_conveyor_button)){
      manual_conveyor = false;
      conveyor_speed=0;
    }

    //Manually control conveyor backwards
    if (joystick_0.getRawButton(Constants.manual_rev_conveyor_button)){
      conveyor_speed = -Constants.manual_conveyor_speed;
      manual_conveyor = true;
    }

    //Stop conveyor when not manually controlled backwards
    if (joystick_0.getRawButtonReleased(Constants.manual_rev_conveyor_button)){
      manual_conveyor = false;
      conveyor_speed=0;
    }

    //Manually control climber
    if (joystick_0.getRawButton(Constants.manual_climber_button)){
      left_climber_speed = Constants.manual_left_climber_speed_up;
      right_climber_speed = Constants.manual_right_climber_speed_up;
      state=0;
    }

    //Stop climber when not manually controlled
    if (joystick_0.getRawButtonReleased(Constants.manual_climber_button)){
      climber_speed=0;
      state=0;
    }

    //Manually control climber backwards
    if (joystick_0.getRawButton(Constants.manual_rev_climber_button)){
      left_climber_speed = -Constants.manual_left_climber_speed_down;
      right_climber_speed= -Constants.manual_right_climber_speed_down;
      state=0;
    }

    //Stop climber when not manually controlled backwards
    if (joystick_0.getRawButtonReleased(Constants.manual_rev_climber_button)){
      left_climber_speed=0;
      right_climber_speed=0;
      state=0;
    }

    //Manually control left climber
    if (joystick_0.getRawButton(Constants.manual_left_climber_button)){
      left_climber_speed = Constants.manual_left_climber_speed_up;
      state=1;
    }

    //Stop left climber when not manually controlled
    if (joystick_0.getRawButtonReleased(Constants.manual_left_climber_button)){
      left_climber_speed=0;
      state=1;
    }

    //Manually control left climber backwards
    if (joystick_0.getRawButton(Constants.manual_rev_left_climber_button)){
      left_climber_speed = -Constants.manual_left_climber_speed_down;
      state=1;
    }

    //Stop left climber when not manually controlled backwards
    if (joystick_0.getRawButtonReleased(Constants.manual_rev_left_climber_button)){
      left_climber_speed=0;
      state=1;
    }
    
    //Manually control right climber
    if (joystick_0.getRawButton(Constants.manual_right_climber_button)){
      right_climber_speed = Constants.manual_right_climber_speed_up;
      state=2;
    }

    //Stop right climber when not manually controlled
    if (joystick_0.getRawButtonReleased(Constants.manual_right_climber_button)){
      right_climber_speed=0;
      state=2;
    }

    //Manually control right climber backwards
    if (joystick_0.getRawButton(Constants.manual_rev_right_climber_button)){
      right_climber_speed = -Constants.manual_right_climber_speed_down;
      state=2;
    }

    //Stop right climber when not manually controlled backwards
    if (joystick_0.getRawButtonReleased(Constants.manual_rev_right_climber_button)){
      right_climber_speed=0;
      state=2;
    }

    //Executes all speeds
    dt.set_speeds(xSpeed, zRotation);
    shooter.set_speed(shooter_speed);
    conveyor.set_speed(conveyor_speed);
    intake.set_speed(intake_speed);

    if(state==0){
      climber.set_speeds(left_climber_speed, right_climber_speed);
    }else if(state==1){
      climber.set_speeds(left_climber_speed,0);
    }else if(state==2){
      climber.set_speeds(0, right_climber_speed);
    }
  }
}
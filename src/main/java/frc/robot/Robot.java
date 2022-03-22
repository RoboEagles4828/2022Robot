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
  double left_climber_speed = 0;
  double right_climber_speed = 0;
  boolean left_can_climb = true;//Hall effects allow to climb up on left
  boolean right_can_climb = true;//Hall effects allow to climb up on right
  int state = 0;//0 is both, 1 is left, 2 is right

  DigitalInput prox_lower = new DigitalInput(Constants.proxy_lower);
  DigitalInput prox_upper = new DigitalInput(Constants.proxy_upper);
  DigitalInput hall_left = new DigitalInput(Constants.hall_left);
  DigitalInput hall_right = new DigitalInput(Constants.hall_right);

  Servo intake_servo = new Servo(Constants.intake_servo_port);
  PWM shooter_servo_0 = new PWM(Constants.shooter_servo_0_port);
  PWM shooter_servo_1 = new PWM(Constants.shooter_servo_1_port);

  Joystick joystick_0 = new Joystick(Constants.joystick_0_port);
  Joystick joystick_1 = new Joystick(Constants.joystick_1_port);

  AHRS navx = new AHRS();

  static final String DefaultAuto = "Default";
  static final String BasicAuto = "DriveShoot";
  static final String DriveAuto = "Drive";
  static final String BottomBallAuto = "BottomBallAuto";//Climbing area is "top" part
  static final String TopBallAuto = "TopBallAuto";
  static final String Shoot = "ShootAuto";
  SendableChooser<String> chooser = new SendableChooser<>();
  String autoSelected;
  Timer timer = new Timer();
  double starting_pos = 0;
  int auto_state = 0;
  double auto_start_time = 0;//Only used for auto shooting
  double start_time = 0;//starting time for auto and recycled for auto shoot time
  boolean first = true;

  double stop_time = 0;//stop drive time
  double conveyor_index_time = 0;
  boolean is_spacing = false;

  @Override
  public void robotInit(){
    chooser.setDefaultOption("Default Auto", DefaultAuto);
    chooser.addOption("Basic Auto", BasicAuto);
    chooser.addOption("Drive Auto", DriveAuto);
    chooser.addOption("Top Ball Auto", TopBallAuto);
    chooser.addOption("Bottom Ball Auto", BottomBallAuto);
    chooser.addOption("Shoot Auto", Shoot);
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
    navx.reset();
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
      case TopBallAuto:
        switch(auto_state){
          case 0:
            System.out.println("0");
            if(timer.get()<Constants.wall_shoot_time){//Shoot at wall
              shooter_speed=Constants.wall_shooter_speed;
              manual_shooter=true;
              if(timer.get()>Constants.conveyor_first_delay){
                conveyor_speed=Constants.conveyor_shoot_speed;
                manual_conveyor=true;
              }
            }else{
              conveyor_speed=0;
              shooter_speed=0;
              manual_conveyor=false;
              manual_shooter=false;
              starting_pos=dt.front_left.getSelectedSensorPosition();
              auto_state++;
            }
            break;
          case 1://Drive to ball
            System.out.println("1");
            if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Constants.top_ball_to_wall*Constants.encoder_ratio){
              xSpeed=Constants.auto_drive_speed;
              intake_speed=Constants.auto_intake_speed;
            }else{
              xSpeed = 0;
              stop_time=timer.get();
              auto_state++;
            }
            break;
          case 2://Stop
            System.out.println("2");
            if(timer.get()-stop_time<Constants.stop_dt_time){
              xSpeed=-0.5;
            }else{
              auto_state++;
              xSpeed=0;
              starting_pos=dt.front_left.getSelectedSensorPosition();
              intake_speed=0;
            }
            break;
          case 3://Drive back
            System.out.println("3");
            if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Constants.top_ball_to_wall*Constants.encoder_ratio){
              xSpeed=-Constants.auto_drive_speed;
            }else{
              auto_state++;
              stop_time=timer.get();
              xSpeed=0;
            }
            break;
          case 4://Stop
            System.out.println("4");
            if(timer.get()-stop_time<Constants.stop_dt_time){
              xSpeed=0.5;
            }else{
              auto_state++;
              start_time=timer.get();
              xSpeed=0;
            }
            break;
          case 5://Shoot again
            System.out.println("5");
            if(timer.get()-start_time<Constants.wall_shoot_time){
              shooter_speed=Constants.wall_shooter_speed;
              manual_shooter=true;
              if(timer.get()-start_time>Constants.conveyor_first_delay){
                conveyor_speed=Constants.conveyor_shoot_speed;
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
      case BottomBallAuto:
        switch(auto_state){
          case 0://Drive Forward
            if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Constants.bottom_tarmac_to_ball*Constants.encoder_ratio){
              xSpeed=Constants.auto_drive_speed;
              intake_speed=Constants.auto_intake_speed;
            }else{
              xSpeed = 0;
              starting_pos=dt.front_left.getSelectedSensorPosition();
              auto_state++;
            }
            break;
          case 1://Drive Back
            if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Constants.bottom_ball_to_pivot*Constants.encoder_ratio){
              xSpeed=-Constants.auto_drive_speed;
              intake_speed=Constants.auto_intake_speed;
            }else{
              xSpeed = 0;
              stop_time=timer.get();
              auto_state++;
            }
            break;
          case 2://Stop
            if(timer.get()-stop_time<Constants.stop_dt_time){
              xSpeed=0.5;
            }else{
              auto_state++;
              xSpeed=0;
              intake_speed=0;
              navx.reset();//May not need this, requires some testing
            }
            break;
          case 3://Turn
            if(navx.getAngle()%360<=Constants.bottom_pivot_angle){
              zRotation=Constants.auto_turn_speed;
            }else{
              zRotation=0;
              auto_state++;
              starting_pos=dt.front_left.getSelectedSensorPosition();
            }
            break;
          case 4://Drive to wall
            if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Constants.bottom_pivot_to_wall*Constants.encoder_ratio){
              xSpeed=-Constants.auto_drive_speed;
            }else{
              xSpeed = 0;
              stop_time=timer.get();
              auto_state++;
            }
            break;
          case 5://Stop
            if(timer.get()-stop_time<Constants.stop_dt_time){
              xSpeed=0.5;
            }else{
              auto_state++;
              start_time=timer.get();
              auto_start_time=timer.get();
              xSpeed=0;
            }
            break;
          case 6://Shoot
            if(timer.get()-start_time<Constants.wall_shoot_time){
              shooter_speed=Constants.wall_shooter_speed;
              if(timer.get()-auto_start_time>Constants.conveyor_delay){
                conveyor_speed=Constants.manual_conveyor_speed;
                manual_conveyor=true;
              }
              if(!detected&&!prox_upper.get()){//Detects a ball as first ball
                detected=true;
              }else if(detected&&prox_upper.get()){//Not at proxy and has been detected, aka the first ball left
                auto_start_time=timer.get();
                detected=false;
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
      /*case BasicAuto:
          switch(auto_state){
            case 0://Drive forward
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Constants.wall_to_ball*Constants.encoder_ratio){
                xSpeed=Constants.auto_drive_speed;
                intake_speed=Constants.auto_intake_speed;
              }else{
                xSpeed = 0;
                stop_time=timer.get();
                auto_state++;
              }
              break;
            case 1://Stop driving
              if(timer.get()-stop_time<Constants.stop_dt_time){
                xSpeed=-0.5;
              }else{
                auto_state++;
                start_time=timer.get();
                xSpeed=0;
              }
              break;
            case 2://Intake ball
              if(timer.get()-start_time<Constants.intake_ball_time){
                intake_speed=Constants.auto_intake_speed;
              }else{
                intake_speed=0;
                starting_pos=dt.front_left.getSelectedSensorPosition();
                auto_state++;
              }
              break;
            case 3://Drive back
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Constants.ball_to_wall*Constants.encoder_ratio){
                xSpeed = -Constants.auto_drive_speed;
              }else{
                xSpeed=0;
                start_time=timer.get();
                auto_state++;
              }
              break;
            case 4://Shoot
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
        break;*/
      case DriveAuto:
          switch(auto_state){
            case 0:
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Constants.default_drive*Constants.encoder_ratio){
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
      case Shoot:
        if(timer.get()<Constants.wall_shoot_time){
          shooter_speed=Constants.wall_shooter_speed;
          manual_shooter=true;
          if(timer.get()>Constants.conveyor_first_delay){
            conveyor_speed=Constants.conveyor_shoot_speed;
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
              if(timer.get()<Constants.wall_shoot_time){
                shooter_speed=Constants.wall_shooter_speed;
                manual_shooter=true;
                if(timer.get()>Constants.conveyor_first_delay){
                  conveyor_speed=Constants.conveyor_shoot_speed;
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
              if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Constants.default_drive*Constants.encoder_ratio){
                xSpeed=Constants.auto_drive_speed;
              }else{
                xSpeed = 0;
                starting_pos=dt.front_left.getSelectedSensorPosition();
                stop_time=timer.get();
                auto_state++;
              }
              break;
            case 2:
              if(timer.get()-stop_time<Constants.stop_dt_time){
                xSpeed=-0.5;
              }else{
                auto_state++;
                xSpeed=0;
              }
              break;
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
    start_time = 0;
    stop_time=0;
    intake_speed=0;
    shooter_speed=0;
    conveyor_speed=0;
    timer.reset();
    timer.start();
    first=true;
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
    zRotation = joystick_1.getRawAxis(2)*0.55; // WPI Drivetrain uses positive=> right

    //All automated functions run first
    //If ball detected make conveyor move to pull it
    if(prox_lower.get()){
      conveyor_speed=Constants.index_conveyor_speed;
      conveyor_index_time=timer.get();
    }else if(!manual_conveyor&&timer.get()-conveyor_index_time>Constants.index_time){//If ball is not detected, so long as it is not under manual control conveyor will stop
      conveyor_speed=0;//Greater than 0.25 seconds since detecting a ball
    }

    //If ball is detected by upper proxy stop conveyor
    if(!prox_upper.get()&&!manual_shooter){//Not upper because for some dumb reason the proxy is switched on robot
      conveyor_speed=0;
    }

    if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)<Constants.encoder_ratio*Constants.spacing&&is_spacing){
      xSpeed=Constants.spacing_speed;
    }


    //All shooter controls
    if(joystick_0.getRawButtonPressed(Constants.manual_shoot_button)){
      start_time=timer.get();
      starting_pos=dt.front_left.getSelectedSensorPosition();
      is_spacing=true;
    }
 
    //Manually control shooter
    if (joystick_0.getRawButton(Constants.manual_shoot_button)){
      shooter_speed=Constants.shooter_volt;
      if(first){
        if(timer.get()-start_time>=Constants.conveyor_first_delay){
          conveyor_speed=Constants.conveyor_shoot_speed;
          manual_conveyor=true;
        }else{
          conveyor_speed=0;
          manual_conveyor=false;
        }
      }else{
        if(timer.get()-start_time>=Constants.conveyor_delay){
          conveyor_speed=Constants.conveyor_shoot_speed;
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
    }

    //Stop shooter when not manually controlled
    if (joystick_0.getRawButtonReleased(Constants.manual_shoot_button)){
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


    //All servo controls
    if(joystick_1.getRawButtonPressed(Constants.intake_servo_button)){
      if(intake_servo.get()>0){
        intake_servo.set(0);
      }else{
        intake_servo.set(1);
      }
    }

    if(joystick_1.getRawButtonPressed(Constants.shooter_servo_button)){
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
    if (joystick_1.getRawButton(Constants.manual_intake_button)){
      intake_speed=Constants.manual_intake_speed;
    }

    //Stop intake when not manually controlled
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


    //Conveyor Controls
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


    //Climber Controls
    //Manually control climber
    if (joystick_0.getRawButton(Constants.manual_climber_button)){
      if(left_can_climb){
        left_climber_speed = Constants.manual_left_climber_speed_up;
      }else{
        left_climber_speed=0;
      }
      if(right_can_climb){
        right_climber_speed = Constants.manual_right_climber_speed_up;
      }else{
        right_climber_speed=0;
      }
      state=0;
    }

    //Stop climber when not manually controlled
    if (joystick_0.getRawButtonReleased(Constants.manual_climber_button)){
      left_climber_speed=0;
      right_climber_speed=0;
      state=0;
    }

    //Manually control climber backwards
    if (joystick_0.getRawButton(Constants.manual_rev_climber_button)){
      left_climber_speed = -Constants.manual_left_climber_speed_down;
      right_climber_speed= -Constants.manual_right_climber_speed_down;
      left_can_climb=true;
      right_can_climb=true;
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
      if(left_can_climb){
        left_climber_speed = Constants.manual_left_climber_speed_up;
      }else{
        left_climber_speed=0;
      }
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
      left_can_climb=true;
      state=1;
    }

    //Stop left climber when not manually controlled backwards
    if (joystick_0.getRawButtonReleased(Constants.manual_rev_left_climber_button)){
      left_climber_speed=0;
      state=1;
    }
    
    //Manually control right climber
    if (joystick_0.getRawButton(Constants.manual_right_climber_button)){
      if(right_can_climb){
        right_climber_speed = Constants.manual_right_climber_speed_up;
      }else{
        right_climber_speed=0;
      }
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
      right_can_climb=true;
      state=2;
    }

    //Stop right climber when not manually controlled backwards
    if (joystick_0.getRawButtonReleased(Constants.manual_rev_right_climber_button)){
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
    

    //Executes all speeds
    dt.set_speeds(xSpeed, zRotation);
    shooter.shooter.setVoltage(shooter_speed);
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
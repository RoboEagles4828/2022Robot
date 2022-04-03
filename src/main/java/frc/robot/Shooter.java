package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.Speeds;

public class Shooter {
    public WPI_TalonSRX shooter;
    private double previous_vel = 0;
    private int valid_iterations=0;

    public Shooter(int port){
        shooter = new WPI_TalonSRX(port);
        shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }

    public void set_speed(double speed){
        shooter.set(speed*-1);
    }

    public void set_voltage(double voltage){
        shooter.setVoltage(voltage);
    }

    public double get_velocity(){
        return shooter.getSelectedSensorVelocity();
    }

    public double get_pos(){
        return shooter.getSelectedSensorPosition();
    }

    public boolean is_ready(){//Once the motor velocity stabilizes and stops oscilating
        double current_vel=get_velocity();
        if(Math.abs(current_vel-previous_vel)<=100){
            valid_iterations++;
        }else{
            valid_iterations=0;
        }
        previous_vel=current_vel;
        if(valid_iterations>10){
            return true;
        }
        return false;
    }

    public boolean reset_is_ready(){
        valid_iterations=0;
        previous_vel=0;
        return false;
    }

    public boolean is_ready(double vel){
        if(get_velocity()>vel){
            return true;
        }
        return false;
    }

    public double get_voltage(){
        return -shooter.getMotorOutputVoltage();
    }

    public double get_vel_threshold(boolean front){
        if(front){
            //System.out.println(-Speeds.vel_threshold*(2837.05*get_voltage()-1140.09));
            return (Speeds.vel_threshold-0.00)*(2837.05*get_voltage()-1140.09);//tested equation for front shooter
            //0.04 is so that the bigger wheel is less sensitive because it is the limiting factor for when to shoot
        }
        //System.out.println(-Speeds.vel_threshold*(3058.85*get_voltage()-2225.71));
        return (Speeds.vel_threshold-0.01)*(3058.85*get_voltage()-2225.71);//tested equation for back shooter
    }
    public void stop(){
        shooter.set(0);
    }
}

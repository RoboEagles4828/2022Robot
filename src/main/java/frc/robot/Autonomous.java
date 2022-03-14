package frc.robot;

public class Autonomous {
    //Drive function, distance in inches
    public static boolean drive(double distance, DriveTrain dt, double starting_pos, int dir){//Return true when reached said distance
        double dist = distance*Constants.encoder_ratio;
        dt.set_speeds(Constants.auto_drive_speed*dir, 0);
        if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)>=dist){
            dt.stop();
            return true;
        }
        return false;
    }
    //Direciton is 1 for forward, -1 for back
    public static boolean intake_drive(double distance, Intake intake, DriveTrain dt, double starting_pos, int dir){
        double dist = distance*Constants.encoder_ratio;
        dt.set_speeds(Constants.auto_drive_speed*dir, 0);
        intake.set_speed(Constants.auto_intake_speed);
        if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)>=dist){
            dt.stop();
            intake.stop();
            return true;
        }
        return false;
    }

    public static boolean intake_drive(double distance, Intake intake, DriveTrain dt, double starting_pos, int dir, boolean stop){
        double dist = distance*Constants.encoder_ratio;
        dt.set_speeds(Constants.auto_drive_speed*dir, 0);
        intake.set_speed(Constants.auto_intake_speed);
        if(Math.abs(dt.front_left.getSelectedSensorPosition()-starting_pos)>=dist){
            dt.stop();
            if(stop){
                intake.stop();
            }
            return true;
        }
        return false;
    }

    public static boolean intake(double time, double start_time, double current_time, Intake intake){
        intake.set_speed(Constants.auto_intake_speed);
        if(current_time-start_time>=time){
            intake.stop();
            return true;
        }
        return false;
    }
    //I dont know which is better, have to test
    public static boolean intake(double time, Intake intake, Conveyor conveyor){
        intake.set_speed(Constants.auto_intake_speed);
        if(conveyor.conveyor.get()!=0){
            intake.stop();
            return true;
        }
        return false;
    } 

    public static boolean shoot(double time, double start_time, double current_time, Shooter shooter){
        shooter.set_speed(Constants.wall_shooter_speed);
        if(current_time-start_time>=time){
            shooter.stop();
            return true;
        }
        return false;
    } 
}

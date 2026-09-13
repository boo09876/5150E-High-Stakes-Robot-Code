#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.h"
#include "pros/vision.hpp"
#include <cmath>
#include <string>

pros::Controller master(pros::E_CONTROLLER_MASTER);

/*** MOTORS ***/
pros::Motor FL(3,false);
pros::Motor BL(15,false);

pros::Motor FR(7,true);
pros::Motor BR(16,true);

pros::Motor intake(10,true);
pros::Motor lift(4,false); 

//Drivetrain motor groups
pros::Motor_Group leftDb({FL, BL});
pros::Motor_Group rightDb({FR, BR});

// TEST FULL DB CHASSIS MOVEMENT
pros::Motor_Group fullChassis({FL, BL, FR, BR});


/*** SENSORS ***/
pros::Imu Gyro(21); // port for inertial
pros::Rotation lift_rot_sensor(19);
//pros::Rotation auton_selector(15);
pros::Optical color_sensor(6);
pros::Distance distance_sensor(5);
pros::Rotation intake_sensor(12);
pros::Vision vision_sensor(20);



/*** PISTONS ***/
pros::ADIDigitalOut mogo('a', LOW);
pros::ADIDigitalOut intakelift('b', LOW);
pros::ADIDigitalOut sweeper('c', LOW);



/*** VARIABLE DECLARATIONS ***/
bool inAuton;
bool mogo_flag;
bool in_loading_position;
bool in_scoring_position;
bool sweeper_flag;
bool unjam_intake;
bool move_to_goal;
bool currently_unjamming;
bool rush_clamp;
bool move_straight_line;
bool hold_ring;
bool reset_lift;
bool double_ring;
bool color_sort;

int loading_position;
int scoring_position;
int clamp_distance;
int start_time;
int move_to_goal_timeout;
int torque_count;
int move_to_goal_speed;

double lift_rotation_kp;
double left_total_velo;
double right_total_velo;

std::string ring_color;
std::string alliance_color;

pros::vision_signature_s_t red_ring_sig = pros::Vision::signature_from_utility(1, 1821, 9669, 5745, -753, 309, -222, 1, 0);
pros::vision_signature_s_t blue_ring_sig = pros::Vision::signature_from_utility(2, -4401, -335, -2368, 1489, 7581, 4535, 1.000, 0);
pros::vision_signature_s_t mogo_sig = pros::Vision::signature_from_utility(1, -373, 301, -36, -4311, -281, -2296, 0.500, 0);



/*** LEMLIB PID ***/
lemlib::Drivetrain drivetrain {
    &leftDb, // left drivetrain motors
    &rightDb, // right drivetrain motors
    10, // track width
    2.75, // wheel diameter
    450, // wheel rpm
    2
    };
 


// Odom construct
    lemlib::OdomSensors sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &Gyro // inertial sensor
    };


// forward/backward PID
    lemlib::ControllerSettings lateralController {
      //kp 9.71 kd 65 - good values
      // 10, 3
    17, // kPs
    0, //kI
    100, // kD
    0, //windupRange
    0.5, // smallErrorRange
    250, // smallErrorTimeout
    1, // largeErrorRange
    500, // largeErrorTimeout
    127 // slew rate
    };
 
// turning PID wow
    lemlib::ControllerSettings angularController {
      // kp: 5.1 kd:32.5
    7.7, // kP
    0, //kI
    50, // kD 
    0, //windupRange
    1, // smallErrorRange
    250, // smallErrorTimeout
    2, // largeErrorRange
    500, // largeErrorTimeout
    80// slew rate
    };


// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);



// Random turn function
void turn(double theta){
    double x = 10000 * (sin(theta * (M_PI / 180.0) + chassis.getPose().x));
    double y = 10000 * (cos(theta * (M_PI / 180.0) + chassis.getPose().y));
    chassis.turnTo(x,y,1000);


}


void screen() {
    while (true) {
        auto pose = chassis.getPose();

        pros::lcd::print(0, "X: %f", pose.x);
        pros::lcd::print(1, "Y: %f", pose.y);
        pros::lcd::print(2, "Heading: %f", pose.theta);
        pros::lcd::print(3, "Alliance Color %s", alliance_color);
        pros::lcd::print(4, "Ring Color: %s", ring_color);
        pros::lcd::print(5, "Temperature: %f", intake.get_temperature());
        pros::lcd::print(6, "Distance: %i", distance_sensor.get());
        //pros::lcd::print(7, "Velocity: %f", intake.get_actual_velocity());
        pros::lcd::print(7, "Rotation: %i", lift_rot_sensor.get_position());
        //pros::lcd::print(7, "Num Objects: %i", vision_sensor.get_object_count());
        //pros::lcd::print(7, "Torque: %f", intake.get_torque());
        //pros::lcd::print(7, "Hue: %f", color_sensor.get_hue());


        


        pros::delay(20);
    }
}



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  /*** INITIALIZE VARIABLES ***/
  inAuton = false;
  mogo_flag = false;
  in_loading_position = false;
  in_scoring_position = false;
  sweeper_flag = false;
  unjam_intake = true;
  move_to_goal = false;
  currently_unjamming = false;
  rush_clamp = false;
  hold_ring = false;
  reset_lift = false;
  double_ring = false;
  color_sort = true;

  loading_position = 3000;
  scoring_position = 20500;
  clamp_distance = 67;
  start_time = 0;
  move_to_goal_timeout = 1000;
  torque_count = 0;
  move_to_goal_speed = 80;

  lift_rotation_kp = 0.027;
  left_total_velo = 1;
  right_total_velo = 1;

  ring_color = "none";
  alliance_color = "none";

  pros::lcd::initialize(); 
  pros::Task screenDisplay(screen);
  chassis.calibrate();
  //lift_rot_sensor.set_position(0);

  lift_rot_sensor.reset_position();
  lift_rot_sensor.set_reversed(true);
  
  color_sensor.set_integration_time(5);
  color_sensor.set_led_pwm(100);
  intake_sensor.set_data_rate(10);



  
  // Detect Ring Color Task
  pros::Task findRingColor([&](){
      while(true){
          if(color_sensor.get_proximity() > 100){
              if(color_sensor.get_rgb().red > color_sensor.get_rgb().blue * 2){  //hue < 70
                  ring_color = "red";
              }
              else if(color_sensor.get_rgb().blue > color_sensor.get_rgb().red * 1.7){ // hue > 90
                  ring_color = "blue";
              }
              else{
                  ring_color = "none";
              }
          }
          else{
              ring_color = "none";
          }
          pros::delay(20);
      }
  }); 
  


  // Color Sort Task
  pros::Task colorSort([&](){
      while(true){
          if(inAuton && 
          ((ring_color == "red" && alliance_color == "blue") || (ring_color == "blue" && alliance_color == "red")) &&
          color_sort){
              pros::delay(100);
              intake = 127;
              pros::delay(100);
              intake = -127;
          }
          pros::delay(20);
      }
  }); 




  // Goal Detection for rushes
  pros::Task clampGoal([&](){
      while(true){
          if(!mogo_flag && rush_clamp && distance_sensor.get() < clamp_distance){
              mogo_flag = true;
              mogo.set_value(HIGH);
              rush_clamp = false;
          }
          pros::delay(20);
      }
  });


  // Goal Detection v2 working
  pros::Task clampGoalv2([&](){
      while(true){
          if(inAuton && move_to_goal){
          while(!mogo_flag && pros::millis() - start_time < move_to_goal_timeout){
              if(distance_sensor.get() > 100){
                left_total_velo += (FL.get_actual_velocity() 
                                    + BL.get_actual_velocity()) / 2;
                right_total_velo += (FR.get_actual_velocity() 
                                    + BR.get_actual_velocity()) / 2;

                leftDb = -move_to_goal_speed * std::pow(right_total_velo / left_total_velo, 2);
                rightDb = -move_to_goal_speed * std::pow(left_total_velo / right_total_velo, 2);     
                
              }
              else{
                fullChassis = -(distance_sensor.get() / 1.3 + 25);
              }
              if(distance_sensor.get() < clamp_distance){
                mogo_flag = true;
                mogo.set_value(HIGH);
              }
          }
            move_to_goal = false;
            fullChassis = 0;
            left_total_velo = 1;
            right_total_velo = 1;
          }
          pros::delay(20);
      }
  });


  // Unjams Intake When Stuck
  pros::Task unstuckIntake([&](){
      while(true){
          if(!in_loading_position && unjam_intake){
              if(intake.get_torque() > 1.0){
                  torque_count++;
              }
              else{
                  torque_count = 0;
              }
              if(torque_count > 15){
                intake = 127;
                torque_count = 0;
                currently_unjamming = true;
                pros::delay(300);
                intake = -127;
                currently_unjamming = false;
                
              }
          }
          pros::delay(20);
      }
  }); 


  // Loads the lift
  pros::Task loadLift([&](){
      while(true){
          if(in_loading_position && !in_scoring_position){
              double error = loading_position - lift_rot_sensor.get_position();
              if (std::abs(error) > 25){
                if (std::abs(lift_rotation_kp * error) > 60){
                    lift = 60 * (error / std::abs(error));
                }
                else{
                    lift = (lift_rotation_kp * error);
                }
                
              }
              else{
                  lift = 0;
              }
          }
        pros::delay(20);
      }
  });



  // Score On Stake
  pros::Task scoreStake([&](){
      while(true){
          if(in_scoring_position){
            double error = scoring_position - lift_rot_sensor.get_position();
              if(std::abs(error) > 25){
                if(lift.get_torque() > 0.5 && lift.get_position() < 4000){
                    intake = 100;
                }
                
                if (lift_rotation_kp * error > 80){
                    lift = 80 * (lift_rotation_kp * 40);
                }
                else{
                    lift = lift_rotation_kp * error;
                }
              }
              else{
                lift = 0;
                intake = 0;
              }
          }
          else if (double_ring){
            double error = 5000 - lift_rot_sensor.get_position();
              if(std::abs(error) > 25){
                if(lift.get_torque() > 0.5 && lift.get_position() < 4000){
                    intake = 100;
                }
                else{
                    lift = lift_rotation_kp * error;
                }
                
              }
              else{
                lift = 0;
              }
          }
          pros::delay(20);
      }
  });



  // Hold Ring In Intake
  pros::Task holdRing([&](){
      while(true){
          if(inAuton && hold_ring && color_sensor.get_proximity() > 100){
              intake = 127;
              pros::delay(200);
              intake = 0;
          }
          pros::delay(20);
      }
  });


  // Reset Lift to Starting Position
  pros::Task liftToStart([&](){
      while(true){
          if(reset_lift){
            double error = 0 - lift_rot_sensor.get_position();
                if(std::abs(error) > 300){
                    lift = (lift_rotation_kp * error) / 2;
                }
                else{
                    lift = 0;
              }
          }
        pros::delay(20);
      }
  });


}

  

// *************** AUTONS **********************//
void skillsAutonv3(){ // WE NEED TO UPDATE THIS VERSION, OLD VERSION FROM WPI

  // SETUP 5 inch from the front of the bot
  intake = -127;
  pros::delay(700);
  intake = 0;

  chassis.moveToPoint(0, 11.5, 1000);
  chassis.turnTo(1000, 11.5, 800, false, 127, false);
  pros::delay(1200);

  move_to_goal_speed = 100;
  start_time = pros::millis();
  move_to_goal = true;
  pros::delay(1100);

  chassis.turnTo(18.3, 25.5, 800);
  intake = -127;
  chassis.moveToPoint(17, 21, 1000);
  chassis.moveToPoint(18.3, 25.5, 1000, true, 80);
  chassis.moveToPoint(24, 36.3, 1000);


  chassis.moveToPoint(51.5, 54.5, 1500, true, 80, false);
  pros::delay(800);

  chassis.turnTo(51, 42, 1000, true, 80);
  chassis.moveToPoint(51, 42, 1000, true, 80);
  pros::delay(2000);

  chassis.moveToPoint(48.2, 20, 2000, true, 60);
  pros::delay(3000);

  chassis.moveToPoint(48.5, 4, 1000, true, 60, false);
  pros::delay(2000);

  chassis.moveToPoint(41.45, 12, 1000, false, 80);
  
  chassis.moveToPoint(53.5, 17, 1000, true, 70);
  pros::delay(800);

  chassis.turnTo(56, 2, 1000, false);
  chassis.moveToPoint(56, 2, 1000, false);
  
  mogo.set_value(LOW);
  pros::delay(400);
  intake = 0;

  chassis.moveToPoint(0, 8.5, 2000);
  chassis.moveToPose(0, 8.5, 96, 2500);
  pros::delay(3000);

  mogo_flag = false;
  rush_clamp = true;
  start_time = pros::millis();
  move_to_goal = true;

  pros::delay(2000);
  chassis.moveToPoint(-80, 0, 2000, false, 80, false);
  mogo.set_value(LOW);
  pros::delay(500);
  chassis.moveToPoint(-50, 10, 1000);


  /*mogo_flag = false;
  rush_clamp = true;
  start_time = pros::millis();
  while(!mogo_flag && pros::millis() - start_time < 1500){
    fullChassis = -60;
  }
  fullChassis = -20;
  pros::delay(500);



  chassis.turnTo(-21.5, 26.5, 800);
  intake = -127;
  chassis.moveToPoint(-20, 22, 1000);
  chassis.moveToPoint(-21.5, 26.5, 1000, true, 80);

  chassis.moveToPoint(-34.5, 46, 1000);

  chassis.moveToPoint(-54, 57, 2000, true, 80, false);
  pros::delay(800);

  chassis.turnTo(-50, 47, 1000, true, 80);
  chassis.moveToPoint(-50, 47, 1000, true, 80, false);
  pros::delay(1500);

  chassis.moveToPoint(-46.5, 20, 1500, true, 60, false);
  pros::delay(1500);

  chassis.moveToPoint(-47, 7, 1500, true, 60, false);
  pros::delay(800);

  chassis.moveToPoint(-46, 40, 1000, false);
  
  chassis.turnTo(-64, 11, 1000);
  chassis.moveToPoint(-64, 11, 1500);
  pros::delay(1500);

  chassis.turnTo(-72, 1, 1000, false);
  chassis.moveToPoint(-72, 1, 1500, false, 127, false);
  mogo.set_value(LOW);
  pros::delay(500);


  fullChassis = 50;
  pros::delay(1000);
  fullChassis = 0;

  chassis.moveToPoint(-45, 78, 2000, true, 127, false);
  pros::delay(300);
  intake = 0;

  chassis.turnTo(-18, 112, 1000, false, 127, false);
  pros::delay(1200);
  
  mogo_flag = false;
  move_to_goal_timeout = 1500;
  start_time = pros::millis();
  move_to_goal = true;

  pros::delay(2000);
  chassis.turnTo(-38.77, 121.75, 1000, false);
  chassis.moveToPoint(-38.77, 121.75, 1000, false, 127, false);
  mogo.set_value(LOW);
  
  pros::delay(300);
  chassis.turnTo(-5, 110, 1000, false);
  
  mogo_flag = false;
  move_to_goal_timeout = 2000;
  start_time = pros::millis();
  move_to_goal = true;

  pros::delay(2500);
  intake = -127;

  chassis.moveToPoint(60, 110, 2000, false);
  chassis.moveToPoint(60, 130, 1000, false);
  mogo.set_value(LOW);
  pros::delay(300);
  chassis.moveToPoint(0, 0, 700);*/
} 


void newSkillsAuton(){
    intake = -127;
    pros::delay(500);
    intake = 0;

    chassis.moveToPoint(0, 12.25, 600);
    chassis.turnTo(1000, 12.25, 800, false, 127, false);

    start_time = pros::millis();
    move_to_goal = true;
    pros::delay(1000);

    chassis.turnTo(17, 25, 800);
    chassis.moveToPoint(17, 25, 800);
    intake = -127;
    
    chassis.turnTo(45, 74, 500);
    chassis.moveToPoint(38.5, 74, 1500, true, 127, false);

    chassis.moveToPoint(35, 56, 1000, false);
    chassis.turnTo(54.84, 59, 800, true, 127, false);
    pros::delay(400);
    

    chassis.moveToPoint(54.5, 59.5, 900, true, 70, false);
    
    
    // SCORE WALL STAKE, CUT OUT FOR TESTING
    /*in_loading_position = true;
    pros::delay(800);*/
    fullChassis = 100;
    pros::delay(400);
    fullChassis = 0;

    /*intake = 0;
    scoring_position = 19000;
    in_scoring_position = true;

    pros::delay(1000);*/


    chassis.moveToPoint(39.27, 58.5, 600, false);

    in_scoring_position = false;
    in_loading_position = false;
    reset_lift = true;

    
    chassis.turnTo(43, 5, 700);
    intake = -127;

    chassis.moveToPoint(42.5, 40, 1000, true, 80);
    //pros::delay(1000);

    chassis.moveToPoint(42.5, 18, 1000, true, 60);
    pros::delay(1500);

    chassis.moveToPoint(42.5, 2, 1000, true, 60);
    //pros::delay(1200);

    chassis.turnTo(54, 12, 700); 
    chassis.moveToPoint(54, 12, 1000);
    //pros::delay(1000);

    chassis.turnTo(55, 0.5, 1000, false);
    chassis.moveToPoint(55, 0.5, 400, false);

    mogo.set_value(LOW);
    intake = 0;
    pros::delay(200);

    chassis.moveToPoint(0, 10, 2000);
    chassis.turnTo(-10000, 10, 1000, false, 127, false);

    mogo_flag = false;
    start_time = pros::millis();
    move_to_goal = true;
    pros::delay(1000);

    chassis.turnTo(-25, 29, 700);
    chassis.moveToPoint(-25, 29, 1000);
    intake = -127;

    chassis.turnTo(-52, 77.5, 600);
    chassis.moveToPoint(-46, 77.5, 1800);
    pros::delay(400);

    chassis.moveToPoint(-41, 62, 1000, false);
    
    chassis.turnTo(-67, 61.5, 800);
    pros::delay(400);
    
    //chassis.moveToPoint(-67, 56.87, 1500, true, 80);

    chassis.moveToPose(-67, 61.5, -450, 1500, {.lead = 0.5, .maxSpeed = 80});
    
    /*reset_lift = false;
    in_loading_position = true;*/

    pros::delay(1000);
    fullChassis = 100;
    pros::delay(500);
    fullChassis = 0;
    /*intake = 0;

    in_scoring_position = true;
    pros::delay(700);

    in_scoring_position = false;
    in_loading_position = false;
    reset_lift = true;*/
    

    // *********** RESET POINTS ** //////////////////////

    chassis.setPose(0, 0, -450, false);
    pros::delay(500);


    chassis.moveToPoint(13.5, 0, 1000, false); // 13.5, 0

    chassis.turnTo(16.07, -17.07, 900); // 14, -16
    intake = -127;

    chassis.moveToPoint(16.07, -17.07, 1000, true, 80); // 14, -16
    //pros::delay(1000);

    chassis.moveToPoint(16.3, -43.7, 1000, true, 60); // 14, -38.5
    pros::delay(1500);

    chassis.moveToPoint(16.5, -57, 1000, true, 60); // 14, -50
    //pros::delay(1200);

    chassis.turnTo(4.5, -53, 700); // -3, -47.3
    chassis.moveToPoint(4.5, -53, 1000); // 3, -47.3
    //pros::delay(1000);

    chassis.turnTo(-1, -58, 1000, false); // 2, -51.1
    chassis.moveToPoint(-1, -58, 600, false); // 2, -51.1

    mogo.set_value(LOW);
    intake = 0;
    pros::delay(200);
    reset_lift = false;
    intake = -127;


    chassis.moveToPoint(0, 0, 500);
    chassis.moveToPoint(31, 15, 2000, true, 127, false);
    in_loading_position = true;
    chassis.moveToPoint(33, 17, 500);

    pros::delay(1000);

    chassis.turnTo(55, 40, 1000, false, 127, false);
    mogo_flag = false;
    rush_clamp = true;

    /*chassis.turnTo(54.84, 32.2, 1000, false, 127, false);
    mogo_flag = false;
    
    rush_clamp = true;
    chassis.moveToPoint(54.84, 32.2, 1000, false, 127, false);
    intake = 0;*/

    chassis.moveToPoint(55, 40, 1200, false, 127, false);
    intake = 0;

    if(!mogo_flag){
        mogo.set_value(HIGH);
        pros::delay(200);
    }

    chassis.turnTo(50, 48.75, 800, true, 127, false);
    fullChassis = 100;
    pros::delay(800);
    fullChassis = 0;

    // *************** RESET 2 ******************* //
    chassis.setPose(0, 0, chassis.getPose().theta, false);
    pros::delay(500);
    chassis.moveToPoint(0, -7, 1000, false, 127, false);

    reset_lift = false;
    in_scoring_position = true;
    scoring_position = 19500;

    pros::delay(650);

    chassis.moveToPoint(0, -13.3, 500, false);

    chassis.turnTo(-50, -14, 700);
    intake = -127;

    in_scoring_position = false;
    in_loading_position = false;
    reset_lift = true;

    chassis.moveToPoint(-50, -14, 1200, true, 127, false);
    intake = -127;

    chassis.moveToPoint(-26.55, -11.5, 1000, false);

    chassis.turnTo(-53.4, 1.4, 600);

    chassis.moveToPoint(-53.4, 1.4, 600, true, 127, false);
    sweeper.set_value(HIGH);
    pros::delay(500);

    chassis.moveToPoint(-55, -15, 500); 

            
    chassis.turnTo(-53.4, 8, 800, false, 127, false);
    pros::delay(500);
    mogo.set_value(LOW);
    sweeper.set_value(LOW);

    chassis.moveToPoint(-53.4, 8, 1200, false, 80, false);

    intake = 20;

    chassis.setPose(0, 0, chassis.getPose().theta, false);

    pros::delay(500);

    chassis.moveToPoint(37.14, -21.42, 1000);

    chassis.moveToPoint(80, -3, 1000);

    chassis.moveToPoint(105, 1, 1500);
    fullChassis = 127;
    pros::delay(1000);
    fullChassis = 0;

}




void redGoalRush(){
  
  // Curve motion to rush the 5th goal 
  rush_clamp = true;
  clamp_distance = 60;

  chassis.moveToPoint(2.5, -25, 650, false);
  chassis.moveToPoint(10.2, -44.5, 1000, false, 127, false);

  if(!mogo_flag){ // Manual Control if auto clamp fails
    mogo.set_value(HIGH);
    pros::delay(300);
    mogo_flag = true;
  }

  // Turns towards first ring stack and holds it in the intake
  chassis.turnTo(12.8, -27, 800, true, 127, false);
  intake = -127;

  chassis.moveToPoint(12.8, -27, 1000, true, 127, false);
  hold_ring = true;
  pros::delay(200);
  intake = 0;
  

  chassis.turnTo(4, -24, 800, false, 127, false);

  chassis.moveToPoint(6, -24, 800, false);
  mogo.set_value(LOW); // Drop the first mobile goal
  pros::delay(300);
  intake = 0;
  
  chassis.moveToPoint(12, -23, 1000);
  chassis.turnTo(10000, -23, 1500, false, 127, false);
  
  // Move to clamp the second mobile goal
  mogo_flag = false;
  move_to_goal_speed = 70;
  start_time = pros::millis();
  move_to_goal = true;
  pros::delay(1000);
  
  chassis.turnTo(50.5, -6.5, 700, true, 127, false);
  hold_ring = false;
  intake = -127;
  intakelift.set_value(HIGH);

  // Intake lift goes down to grab the top ring from the middle stack
  chassis.moveToPoint(50.5, -6.5, 1200, true, 127, false);
  intakelift.set_value(LOW);

  fullChassis = 80;
  pros::delay(150);
  fullChassis = 0;

  pros::delay(200);
  unjam_intake = false;


  chassis.moveToPoint(41.8, -13.6, 800, false);
  //chassis.moveToPoint(41, -6.5, 600);
  unjam_intake = true;











  // Goes to touch the ladder; commented parts are for elims
  //chassis.turnTo(38, -27, 800);
  //chassis.moveToPoint(38, -27, 1000, true, 70, false);


  chassis.turnTo(4, -40, 800);
  chassis.moveToPoint(4, -40, 1000, true, 127, false);
}

void blueGoalRush(){

  // Curve Motion to rush the 5th goal
  rush_clamp = true;
  clamp_distance = 60;
  chassis.moveToPoint(-2, -25, 650, false);
  chassis.moveToPoint(-10, -42, 1500, false, 127, false);
  
  if(!mogo_flag){
    mogo.set_value(HIGH);
    pros::delay(300);
    mogo_flag = true;
  }

  // Turns to the first ring stack
  chassis.turnTo(-15, -24, 800, true, 127, false);
  intake = -127;
  chassis.moveToPoint(-15, -24, 1000, true, 127, false);
  hold_ring = true;
  pros::delay(750);
  intake = 0;

  // Drops the mobile goal near the corner 
  chassis.turnTo(-9, -32, 1000, false);
  chassis.moveToPoint(-9, -32, 1000, false, 127, false);
  mogo.set_value(LOW);

  chassis.turnTo(-32, -32, 700);
  chassis.moveToPoint(-15, -32, 600);
  chassis.turnTo(-32, -27.5, 750, false, 127, false);

  // Moves to clamp the next goal
  mogo_flag = false;
  move_to_goal_speed = 60;
  start_time = pros::millis();
  move_to_goal = true;
  pros::delay(1000); 

  chassis.turnTo(-49.5, -11, 750, true, 127, false);
  intakelift.set_value(HIGH);
  hold_ring = false;
  intake = -127;    
  
  // Intake up and down to grab the top ring off the middle stack
  chassis.moveToPoint(-49.5, -11, 1200, true, 127, false);
  intakelift.set_value(LOW);  

  fullChassis = 80;
  pros::delay(150);
  fullChassis = 0;

  pros::delay(250);
  unjam_intake = false;

  

  chassis.moveToPoint(-43, -16, 1000, false);
  //chassis.moveToPoint(-50, -12.5, 800);
  unjam_intake = true;

  // Turn to touch the ladder
  /*chassis.turnTo(-45, -23, 750);
  chassis.moveToPoint(-45, -22.8, 1000, true, 70);
  in_scoring_position = true;*/

  // Elims
  chassis.turnTo(-10, -38, 700);
  chassis.moveToPoint(-10, -38, 1000);



  
}

void redSweepRush(){
    // Curve motion to rush the 5th goal 
    rush_clamp = true;
    clamp_distance = 60;

    chassis.moveToPoint(2.5, -25, 650, false);
    chassis.moveToPoint(10.2, -44.5, 1000, false, 127, false);

    if(!mogo_flag){ // Manual Control if auto clamp fails
        mogo.set_value(HIGH);
        pros::delay(300);
        mogo_flag = true;
    }

    // Turns towards first ring stack and holds it in the intake
    chassis.turnTo(13.5, -26, 800, true, 127, false);
    intake = -127;
    chassis.moveToPoint(13.5, -26, 1000, true, 127, false);
    pros::delay(1500);
    intake = 0;

    chassis.turnTo(45, -4.28, 800, false);
    chassis.moveToPoint(45, -4.28, 1000, false, 127, false);    

    mogo.set_value(LOW);
    intake = 0;
    pros::delay(200);


    chassis.moveToPoint(27.86, -18.35, 1000);

    chassis.turnTo(32.23, -31.5, 1000, false, 127, false);
    mogo_flag = false;
    start_time = pros::millis();
    move_to_goal = true;
    while(!mogo_flag && pros::millis() - start_time < 1000){
        pros::delay(20);
    }
    //chassis.moveToPoint(32.23, -31.5, 1000, false, 127, false);
 

    if(!mogo_flag){ // Manual Control if auto clamp fails
        mogo.set_value(HIGH);
        pros::delay(300);
        mogo_flag = true;
    }

    chassis.moveToPoint(22, -1.27, 1000);
    hold_ring = false;
    intake = -127;

    chassis.turnTo(5.22, 1.6, 800, true, 127, false);
    sweeper.set_value(HIGH);

    chassis.moveToPoint(5.22, 1.6, 1000, true, 127, false);
    fullChassis = 50;
    pros::delay(200);

    leftDb = -127;
    rightDb = 127;

    pros::delay(750);
    fullChassis = 90;
    pros::delay(750);
    fullChassis = -50;
    pros::delay(100);
    fullChassis = 0;
    /*chassis.moveToPoint(30, -5, 1000, false, 127, false);

    mogo.set_value(LOW);
    mogo_flag = false;
    intake = 0;
    pros::delay(200);

    chassis.moveToPoint(21, -9, 600);

    chassis.turnTo(33, -32.5, 600, false);
    rush_clamp = true;
    sweeper.set_value(HIGH);

    chassis.moveToPoint(33, -32.5, 1000, false, 127, false);
    hold_ring = false;
    intake = -127;

    chassis.moveToPoint(5, -2, 1500, true, 100, false);
   
    rightDb = 127;
    leftDb = -127;
    pros::delay(1000);

    sweeper.set_value(LOW);

    chassis.moveToPoint(10, -25, 1000);*/    

}

void blueSweepRush(){
    // Curve Motion to rush the 5th goal
    rush_clamp = true;
    clamp_distance = 60;
    chassis.moveToPoint(-2, -25, 650, false);
    chassis.moveToPoint(-10.2, -43, 1500, false, 127, false);
  
    if(!mogo_flag){
        mogo.set_value(HIGH);
        pros::delay(300);
        mogo_flag = true;
    }

    // Turns to the first ring stack
    chassis.turnTo(-15.5, -24, 800, true, 127, false);
    intake = -127;
    chassis.moveToPoint(-15.5, -24, 1000, true, 127, false);
    hold_ring = true;

    // Drops the mobile goal near the corner 
    chassis.turnTo(-9, -29, 1000, false);
    chassis.moveToPoint(-9, -29, 1000, false, 127, false);
    mogo.set_value(LOW);

    chassis.turnTo(-32, -32, 700);
    chassis.moveToPoint(-15, -32, 600);
    chassis.turnTo(-32, -27.5, 750, false, 127, false);

    // Moves to clamp the next goal
    mogo_flag = false;
    move_to_goal_speed = 60;
    start_time = pros::millis();
    move_to_goal = true;
    pros::delay(1000); 

    chassis.moveToPoint(5, -8, 1000);
    sweeper.set_value(HIGH);
    chassis.turnTo(-32, -32, 1000);
    chassis.turnTo(8.2, -1, 1000);

    chassis.moveToPoint(8.2, -2, 1000);

    chassis.turnTo(0, 2.55, 700);
    chassis.moveToPoint(0, 2.55, 1000);
    sweeper.set_value(LOW);
    hold_ring = false;
    intake = -127;

    chassis.moveToPoint(-4, -40, 1000);
}

void armRush(){ // TEMPORARY TEST AUTO

    // Corner by 9
    hold_ring = true;
    sweeper.set_value(HIGH);
    intake = -127;


    chassis.moveToPoint(0, 35.5, 1000, true, 127, false);
    sweeper.set_value(LOW);
    pros::delay(100);

    chassis.moveToPoint(0, 20, 1000, false, 127, false);
    sweeper.set_value(HIGH);

    chassis.moveToPoint(0, 22, 300);

    chassis.turnTo(6.5, 29.7, 1000, false, 100, false);

    move_to_goal_speed = 70;
    start_time = pros::millis();
    move_to_goal = true;
    pros::delay(1000);

    sweeper.set_value(LOW);
    hold_ring = false;

    chassis.moveToPoint(12.5, 13, 1000);
    intake = -127;
    chassis.moveToPoint(13, 10, 1000);

}





void oldredRingSide(){

  // Moves straight to clamp the goal
  move_to_goal_speed = 70;
  start_time = pros::millis();
  move_to_goal = true;
  pros::delay(1300);
  mogo.set_value(HIGH);

  // Turn to the middle 8 stack and intakes the bottom two
  chassis.turnTo(17.2, -39, 800, true, 127, false);
  intake = -127;
  chassis.moveToPoint(17.2, -39, 1000);
  pros::delay(1000);

  chassis.moveToPoint(10.88, -31.87, 800, false);

  chassis.turnTo(26, -38, 800);
  chassis.moveToPoint(26, -38, 1000);
  pros::delay(1000);


  // Intake the third ring in that area
  chassis.moveToPoint(6, -31.21, 1000, false);
  chassis.turnTo(20, -22.5, 700);
  chassis.moveToPoint(20, -22.5, 1000);
  pros::delay(1500);

  chassis.turnTo(-26, -2.5, 700, true, 127, false);
  intakelift.set_value(HIGH);

  // Goes to middle stack in front of alliance stake, intake lift and down to take top ring
  chassis.moveToPoint(-26, -2.5, 1500, true, 127, false);
  intakelift.set_value(LOW);
  unjam_intake = false;
  
  chassis.moveToPoint(-17, -7.5, 600, false, 127, false);
  chassis.moveToPoint(-22, -4.5, 600);
  unjam_intake = true;
  
  // Turns to touch the ladder
  pros::delay(800);
  chassis.turnTo(-18.5, -17.5, 700);
  chassis.moveToPoint(-18.5, -17.5, 1500, true, 70, false);
  in_scoring_position = true;


  
  //in_scoring_position = true;
  //chassis.moveToPoint(-16, -12.5, 600, true, 70, false);

  /*chassis.turnTo(-15, -28, 700);
  chassis.moveToPoint(-15, -28, 1500, true, 90);
  in_scoring_position = true;
  intake = 0;*/
}

void newRedRingSide(){

    in_loading_position = true;
    chassis.turnTo(4.5, 6.85, 500, true, 127, false);
    intake = -127;
    pros::delay(100);

    chassis.moveToPoint(4.5, 6.85, 500, true, 127, false);
    intake = 0;


    in_scoring_position = true;
    scoring_position = 19000;

    start_time = pros::millis();
    while(lift_rot_sensor.get_position() < 18500 && pros::millis() - start_time < 100){ 
        pros::delay(20);
    }
    rush_clamp = true;


    //**** CHANGE THE TIMEOUT TO 700 ****//

    chassis.moveToPoint(-20, -6.5, 800, false);
    chassis.moveToPoint(-25, -6.5, 1000, false, 127, false);
    in_scoring_position = false;
    in_loading_position = false;
    reset_lift = true;

    if(!mogo_flag){
        mogo.set_value(HIGH);
        pros::delay(200);
    }

    chassis.turnTo(-35, -22.3, 800);

    chassis.moveToPoint(-35, -22.3, 1200);
    intake = -127;
    pros::delay(500);

    chassis.turnTo(-35.2, -34, 800);
    chassis.moveToPoint(-35.2, -34, 1000);
    pros::delay(500);

    chassis.moveToPoint(-23.37, -18, 1000, false);
    chassis.turnTo(-18, -24.7, 700);
    chassis.moveToPoint(-18, -24.7, 1000);
    pros::delay(500);

    chassis.turnTo(-7.5, 10.75, 800, true, 127, false);

    chassis.moveToPoint(-7.5, 10.75, 1000, true, 127, false);
    intakelift.set_value(HIGH);
    unjam_intake = false;

    chassis.moveToPoint(0, 19, 1500, true, 30, false);
    intakelift.set_value(LOW);
    pros::delay(200);

    fullChassis = -80;
    pros::delay(500);
    unjam_intake = true;

    fullChassis = 0;
    pros::delay(200);

    fullChassis = 80;
    pros::delay(300);
    fullChassis = 0;

    scoring_position = 20500;

    /*chassis.turnTo(-35, -33, 700);
    chassis.moveToPoint(-35, -33, 1000);
    pros::delay(2000);
    intake = 0;

    
    chassis.turnTo(-41.2, -1000, 360);
    chassis.moveToPoint(-41, -1000, 550);
    pros::delay(500);

    chassis.moveToPoint(-30, -15.35, 1000, false);

    chassis.turnTo(-23, -25, 500);
    chassis.moveToPoint(-23, -25, 1000);
    pros::delay(1500);

    chassis.turnTo(-4, 15, 800, true, 127, false);
    intake = 0;
    intakelift.set_value(HIGH);

    chassis.moveToPoint(-4, 15, 1500, true, 127, false);
    intakelift.set_value(LOW);
    pros::delay(400);
    intake = -127;

    fullChassis = -80;
    pros::delay(500);
    fullChassis = 80;
    pros::delay(550);
    fullChassis = 0;*/

    scoring_position = 20500;
}

void oldblueRingSide(){ // MOST UPDATED VERSION OF BLUE RING + LINEUP Direct with the mogo one hole over line (red too)

  // Move straight to clamp the mobile goal
  move_to_goal_speed = 70;
  start_time = pros::millis();
  move_to_goal = true;
  pros::delay(1100);

  // Turn to the 8 stack and intake both of them
  chassis.turnTo(-14.5, -51.5, 800, true, 127, false);
  intake = -127;
  chassis.moveToPoint(-14.5, -51.5, 1000);
  pros::delay(1000);

  //chassis.moveToPoint(-9, -44.2, 1000, false);
  chassis.turnTo(-23, -53.5, 600);
  chassis.moveToPoint(-23, -53.5, 1000);
  chassis.moveToPoint(-30, -52.5, 1000);

  pros::delay(1000);

  // Intake the third ring on the ringside chassis.moveToPoint(-6.5, -42.86, 1000, false);
  chassis.turnTo(-23, -38.5, 800);
  chassis.moveToPoint(-23, -38.5, 1000);
  pros::delay(1500);

  chassis.turnTo(13.5, -14.5, 800, true, 127, false);
  intakelift.set_value(HIGH);

  // Intake lift up and down to intake the top ring in front of the alliance stake
  chassis.moveToPoint(13.5, -14.5, 1500, true, 127, false);
  intakelift.set_value(LOW);
  unjam_intake = false;

  chassis.moveToPoint(7, -17, 700, false);
  chassis.moveToPoint(9, -16, 300);
  unjam_intake = true;
  /*chassis.turnTo(-16.5, -49, 800, true, 127, false);  // 16.2 48.5
  intake = -127;
  chassis.moveToPoint(-16.5, -49, 1000);
  pros::delay(1000);

  chassis.moveToPoint(-15, -46, 600, false);
  chassis.turnTo(-21.2, -50.5, 1000);  // 21 50.2
  chassis.moveToPoint(-21.2, -50.5, 1000);
  pros::delay(1000);

  chassis.moveToPoint(-9, -44.2, 1000, false);
  chassis.turnTo(-18.5, -40, 700);
  chassis.moveToPoint(-18.5, -40, 1000);
  pros::delay(1500);

  chassis.turnTo(18.5, -12, 700, true, 127, false);
  intakelift.set_value(HIGH);

  chassis.moveToPoint(18.5, -12, 1500, true, 127, false);
  intakelift.set_value(LOW);
  unjam_intake = false;

  
  chassis.moveToPoint(7, -20.76, 1000, false, 127, true);
  
  chassis.moveToPoint(10, -19.5, 300);
  unjam_intake = true;

  chassis.turnTo(11, -33.3, 700);
  in_scoring_position = true;
  chassis.moveToPoint(11, -33.3, 1000);
 */
}

void newBlueRingSide(){

    in_loading_position = true;
    intake = -127;

    chassis.turnTo(-5, 2.8, 600, true, 127, false);

    chassis.moveToPoint(-5, 2.8, 600, true, 127, false);
    intake = 0;


    in_scoring_position = true;
    scoring_position = 19000;

    start_time = pros::millis();
    while(lift_rot_sensor.get_position() < 18500 && pros::millis() - start_time < 700){
        pros::delay(20);
    }

    rush_clamp = true;
    chassis.moveToPoint(16, -11, 800, false);
    chassis.moveToPoint(26, -12, 1000, false, 127, false);
    in_scoring_position = false;
    in_loading_position = false;
    reset_lift = true;

    if(!mogo_flag){
        mogo.set_value(HIGH);
        pros::delay(200);
    }

    chassis.turnTo(54, -28, 800);
    chassis.moveToPoint(54, -28, 1200);
    intake = -127;
    pros::delay(500);

    chassis.turnTo(57.5, -40, 800);
    chassis.moveToPoint(57.5, -40, 1000);
    pros::delay(500);

    chassis.moveToPoint(45, -18, 1000, false);

    chassis.turnTo(43.7, -31.3, 700);
    chassis.moveToPoint(43.7, -31.3, 1000);
    pros::delay(500);

    chassis.turnTo(28.3, -12.1, 700);
    chassis.moveToPoint(28.3, -12.1, 1000, true, 127, false);
    intakelift.set_value(HIGH);
    unjam_intake = false;

    chassis.moveToPoint(21, 0, 2000, true, 30, false);
    intakelift.set_value(LOW);
    pros::delay(200);

    fullChassis = -80;
    pros::delay(500);
    unjam_intake = true;

    fullChassis = 0;
    pros::delay(200);

    fullChassis = 80;
    pros::delay(300);
    fullChassis = 0;

    scoring_position = 20500;
    /*pros::delay(1500);

    chassis.moveToPoint(52, -1000, 680);
    pros::delay(200);

    chassis.moveToPoint(42, -20, 1000, false);

    chassis.turnTo(42, -35, 800);
    chassis.moveToPoint(42, -35, 1000);
    pros::delay(1500);

    chassis.turnTo(22.6, -5.42, 800);
    intakelift.set_value(HIGH);

    chassis.moveToPoint(17, -3, 1500, true, 127, false);
    intakelift.set_value(LOW);
    pros::delay(400);

    fullChassis = -80;
    pros::delay(500);
    fullChassis = 80;
    pros::delay(500);
    fullChassis = 0;

    scoring_position = 20500;*/
}





void oldRedSolo(){ // RETUNE

    in_loading_position = true;
// Maybe try the curve movement
    chassis.turnTo(4.5, 6.85, 500, true, 127, false);
    intake = -127;
    pros::delay(100);

    chassis.moveToPoint(4.5, 6.85, 500, true, 127, false);
    intake = 0;


    in_scoring_position = true;
    scoring_position = 19000;

    start_time = pros::millis();
    while(lift_rot_sensor.get_position() < 18500 && pros::millis() - start_time < 700){
        pros::delay(20);
    }
    rush_clamp = true;

    chassis.moveToPoint(-20, -6.5, 800, false);
    chassis.moveToPoint(-25, -6.5, 1000, false, 127, false);
    in_scoring_position = false;
    in_loading_position = false;
    reset_lift = true;

    if(!mogo_flag){
        mogo.set_value(HIGH);
        pros::delay(200);
    }

    chassis.turnTo(-37.5, -23, 800);

    chassis.moveToPoint(-37.5, -23, 900);
    intake = -127;


    
    chassis.turnTo(-41.2, -1000, 380);
    chassis.moveToPoint(-41, -1000, 600);
    pros::delay(200);

    chassis.moveToPoint(-30, -15.35, 1000, false);

    chassis.turnTo(-23, -25, 500);
    chassis.moveToPoint(-23, -25, 1000);

    /*chassis.moveToPoint(-2, 35, 1200, false, 127, false);
    mogo.set_value(LOW);
    mogo_flag = false;

    chassis.moveToPoint(-4, 30, 500);
    chassis.turnTo(-17, 45, 600, false);
    chassis.moveToPoint(-17, 45, 800, false);
    rush_clamp = true;

    chassis.turnTo(-23, 71, 600);
    chassis.moveToPoint(-24, 71, 1000);    

    chassis.moveToPoint(-30, 56, 700, false);*/
   

    /*chassis.turnTo(-2, 3.5, 650);
    chassis.moveToPoint(-2, 3.5, 1000);
    

    chassis.turnTo(-2, 1000, 500);
    mogo.set_value(LOW);
    mogo_flag = false;

    hold_ring = true;
    color_sort = false;

    chassis.moveToPoint(-2, 11.5, 700, true, 80);
    pros::delay(300);

    chassis.moveToPoint(-2, 30, 1000);
    

    chassis.turnTo(-17, 42, 600, false);
    chassis.moveToPoint(-17, 42, 1000, false);
    rush_clamp = true;

    chassis.turnTo(-21.55, 61.8, 600);

    chassis.moveToPoint(-21.55, 61.8, 1000);*/


    scoring_position = 20500;
}


void redSoloAWP(){

    in_loading_position = true;
    intake = -127;
    pros::delay(500);

    intake = 0;
    in_scoring_position = true;
    scoring_position = 18500;

    chassis.moveToPoint(0, 3.5, 500, true, 127, false);
    pros::delay(450);

    chassis.moveToPoint(0, -10, 800, false);

    rush_clamp = true;
    chassis.moveToPoint(-10.5, -26.5, 800, false, 127, false);

    if(!mogo_flag){
        mogo.set_value(HIGH);
        mogo_flag = true;
        pros::delay(200);
    }


    in_loading_position = false;
    in_scoring_position = false;
    reset_lift = true;

    chassis.turnTo(-9, -50.87, 800);
    chassis.moveToPoint(-9, -50.87, 1000);
    intake = -127;

    chassis.turnTo(-3.5, -56.5, 700);
    chassis.moveToPoint(-3.5, -56.5, 1000);

    chassis.moveToPoint(-8, -12, 1000, false);

    chassis.moveToPoint(-22, 14.34, 1000, false, 127, false);

    mogo.set_value(LOW);
    intake = 0;
    pros::delay(100);

    chassis.moveToPoint(-22, 9.2, 600);
    chassis.turnTo(-39.5, 9.6, 650, false);
    
    mogo_flag = false;
    rush_clamp = true;

    chassis.moveToPoint(-39.5, 9.6, 800, false);

    chassis.turnTo(-62, 25, 600);
    chassis.moveToPoint(-62, 25, 1000);
    intake = -127;

    chassis.turnTo(-38.62, 3.58, 500, true, 127, false);
    fullChassis = 80;



    

}

void blueSoloAWP(){  // Currently blue rings

    in_loading_position = true;
    intake = -127;
    pros::delay(500);

    intake = 0;
    in_scoring_position = true;
    scoring_position = 19000;

    chassis.moveToPoint(0, 2, 500);
    pros::delay(700);
    chassis.moveToPoint(0, -10, 800, false, 127, false);

    in_loading_position = false;
    in_scoring_position = false;
    reset_lift = true;

    chassis.turnTo(19, -27.9, 600, false);
    rush_clamp = true;

    chassis.moveToPoint(19, -27.9, 1200, false, 100, false);

    if(!mogo_flag){
        mogo.set_value(HIGH);
    }

    pros::delay(300);

    chassis.turnTo(33, -57.3, 750);
    intake = -127;
    chassis.moveToPoint(33, -57.3, 1000, true, 127, false);
    pros::delay(400);

    chassis.turnTo(31.16, -65, 600);
    chassis.moveToPoint(31.16, -65, 1000, true, 127, false);
    
    //chassis.moveToPoint(27, -44.36, 900, false);

    chassis.moveToPoint(27, -40, 900, false);

    chassis.turnTo(22.72, -49, 700);
    chassis.moveToPoint(22.72, -49, 1000, true, 127, false);

    pros::delay(400);

    chassis.turnTo(39, -19.83, 1000, true, 127, false);

    fullChassis = 40;
    pros::delay(1000);
    intake = 0;










    /*chassis.moveToPoint(34.37, 15.62, 1500, false, 127, false);

    mogo.set_value(LOW);
    intake = 0;

    chassis.moveToPoint(34.27, 4.35, 700);
    chassis.turnTo(49.8, 3.5, 700, false);
    mogo_flag = false;
    rush_clamp = true;

    chassis.moveToPoint(49.8, 3.5, 1000, false);

    chassis.turnTo(64.32, 24.79, 700);
    chassis.moveToPoint(64.32, 24.79, 800);
    intake = -127;

    chassis.turnTo(60, 0, 500, true, 127, false);
    fullChassis = 127;*/


    
}




/********* WPI AUTOS ***********/
void safeSoloAWP(){ // WPI AUTO, HAVENT TESTED SINCE THEN
  //chassis.moveToPoint(0, -18, 1000, false, 127, false);

  start_time = pros::millis();
  move_to_goal = true;
  pros::delay(1300);


  chassis.turnTo(-18.5, -30.2, 1000);
  chassis.moveToPoint(-18.5, -30.2, 1000);
  intake = -127;  
  pros::delay(750);

  chassis.moveToPoint(44, -17, 1500, false, 127, false); // 17.5
  mogo.set_value(LOW);
  chassis.moveToPoint(40, -23.5, 800);

  chassis.turnTo(43.5, -27, 750, false, 127, false);

  
  mogo_flag = false;

  move_to_goal_timeout = 1000;
  start_time = pros::millis();
  move_to_goal = true;
  pros::delay(750);
  
  chassis.turnTo(65.5, -29, 750);
  intake = -127;
  chassis.moveToPoint(65.5, -29, 1000);
  pros::delay(750);
  

  chassis.moveToPoint(43.5, -27, 1000, false, 127, false);
  pros::delay(1000);
  chassis.turnTo(43.83, -35, 1000, true, 127, false);
  chassis.moveToPoint(43.83, -35, 1000);
  intake = 0;
  in_scoring_position = true;
}


/********* STATES AUTOS ********/
void statesRedRingSide(){

    in_loading_position = true;
    intake = -127;
    pros::delay(600);
    intake = 0;

    chassis.moveToPoint(0, 3.5, 500);
    in_scoring_position = true;
    scoring_position = 19000;
    pros::delay(750);

    chassis.moveToPoint(0, -4.4, 1000, false, 127, false);

    in_loading_position = false;
    in_scoring_position = false;
    reset_lift = true;

    chassis.turnTo(-15.63, -25.17, 600, false);
    rush_clamp = true;

    chassis.moveToPoint(-15.63, -25.17, 1000, false, 80, false);

    if(!mogo_flag){
        mogo.set_value(HIGH);
        mogo_flag = true;
    }

    pros::delay(200);

    chassis.turnTo(-17, -48.5, 700);
    intake = -127;

    chassis.moveToPoint(-17, -48.5, 1000, true, 127, false);
    pros::delay(300);

    chassis.turnTo(-13, -54, 600);
    chassis.moveToPoint(-13, -54, 1000);

    chassis.moveToPoint(-12.85, -33, 1000, false);

    chassis.turnTo(-4.2, -40.39, 700);
    chassis.moveToPoint(-4.2, -40.39, 1000, true, 127, false);
    pros::delay(300);

    chassis.turnTo(-12.84, 0.86, 800, true, 127, false);
    intakelift.set_value(HIGH);

    chassis.moveToPoint(-12.84, 0.86, 1500, true, 127, false);
    intakelift.set_value(LOW);
    fullChassis = 50;
    pros::delay(200);
    fullChassis = 0;

    chassis.moveToPoint(-11.71, -11.78, 1000, false);
    chassis.turnTo(-21, -9.46, 600);
    chassis.moveToPoint(-21, -9.46, 1500, true, 100, false);

    fullChassis = 50;


    

}

void statesBlueRingSide(){
    in_loading_position = true;
    intake = -127;
    pros::delay(600);
    intake = 0;

    chassis.moveToPoint(0, 3.5, 500);
    in_scoring_position = true;
    scoring_position = 19000;
    pros::delay(750);

    chassis.moveToPoint(0, -4.4, 1000, false, 127, false);

    in_loading_position = false;
    in_scoring_position = false;
    reset_lift = true;

    chassis.turnTo(16.81, -27.33, 600, false);
    rush_clamp = true;

    chassis.moveToPoint(16.81, -27.33, 1000, false, 80, false);

    if(!mogo_flag){
        mogo.set_value(HIGH);
        mogo_flag = true;
    }

    pros::delay(200);

    chassis.turnTo(26, -56.18, 700);
    intake = -127;

    chassis.moveToPoint(26, -56.18, 1000, true, 127, false);
    pros::delay(300);

    chassis.turnTo(20.56, -68, 600);
    chassis.moveToPoint(20.56, -68, 1000);

    chassis.moveToPoint(25.16, -41.5, 1000, false);

    chassis.turnTo(13.13, -52.96, 700);
    chassis.moveToPoint(13.13, -52.96, 1000, true, 127, false);
    pros::delay(300);

    chassis.turnTo(15.94, -20.39, 800, true, 127, false);
    intakelift.set_value(HIGH);

    chassis.moveToPoint(15.94, -20.39, 1500, true, 127, false);
    intakelift.set_value(LOW);
    fullChassis = 50;
    pros::delay(200);
    fullChassis = 0;

    chassis.moveToPoint(12.63, -22.92, 1000, false);
    chassis.turnTo(26.5, -17.12, 600);
    chassis.moveToPoint(26.5, -17.12, 1500, true, 50);

    

}





/******** WORLDS AUTOS ********/

void worldsRedRingSide(){
    
    lift_rot_sensor.set_position(2600);
    in_scoring_position = true;
    scoring_position = 18500;

    chassis.moveToPoint(0, 3.5, 500, true, 127, false);
    pros::delay(450);

    chassis.moveToPoint(0, -28, 1000, false);    

    rush_clamp = true;
    chassis.turnTo(-15, -30, 700, false);
    chassis.moveToPoint(-15, -30, 1000, false, 127, false);


    if(!mogo_flag){
        mogo.set_value(HIGH);
        mogo_flag = true;
        pros::delay(200);
    }


    in_loading_position = false;
    in_scoring_position = false;
    reset_lift = true;


// Middle Rings

    chassis.turnTo(-15, -48.5, 800);
    chassis.moveToPoint(-15, -48.5, 1000);
    intake = -127;
    
    chassis.turnTo(-9, -58, 700);
    chassis.moveToPoint(-9, -58, 1000);
    chassis.moveToPoint(-3, -60, 500);

    pros::delay(500);


    chassis.turnTo(-2.5, -45, 600);
    chassis.moveToPoint(-2.5, -45, 1000, true, 127, false);

    pros::delay(300);

    chassis.moveToPoint(35.75, -31, 1000, true, 127, false);

    fullChassis = 50;
    pros::delay(1000);
    
    fullChassis = -50;
    pros::delay(1000);

    fullChassis = 0;

    chassis.turnTo(-2, -14, 700);
    chassis.moveToPoint(-2, -14, 1000, true, 127, false);

    intakelift.set_value(HIGH);

    chassis.moveToPoint(-8.5, -4, 700, true, 100, false);
    intakelift.set_value(LOW);
    
    fullChassis = -50;
    pros::delay(500);
    fullChassis = 0;

}

void worldsBlueRingSide(){

    lift_rot_sensor.set_position(2600);
    in_scoring_position = true;
    scoring_position = 18500;

    chassis.moveToPoint(0, 3.5, 500, true, 127, false);
    pros::delay(450);

    chassis.moveToPoint(0, -30, 1000, false);  
    
    chassis.turnTo(15, -34, 700, false, 127, false);
    rush_clamp = true;

    chassis.moveToPoint(15, -34, 1000, false, 127, false);


    if(!mogo_flag){
        mogo.set_value(HIGH);
        mogo_flag = true;
        pros::delay(200);
    }


    in_loading_position = false;
    in_scoring_position = false;
    reset_lift = true;

    // Middle Rings

    chassis.turnTo(22, -57.3, 800);
    chassis.moveToPoint(22, -57.3, 1000);
    intake = -127;
    
    chassis.turnTo(17, -66, 700);
    chassis.moveToPoint(17, -66, 1000);
    chassis.moveToPoint(12, -64, 500);

    pros::delay(500);

    chassis.turnTo(7.75, -59.59, 700);
    chassis.moveToPoint(7.75, -59.59, 1000);

    chassis.moveToPoint(-25, -48, 1000, true, 127, false);

    fullChassis = 50;
    pros::delay(1000);
    
    fullChassis = -70;
    pros::delay(600);

    fullChassis = 0;
   
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}


/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
 /***/
 /** auto for league games
 void autonomous() {
    chassis.moveTo(0,-150,600);
    pros::delay(600);
    chassis.moveTo(0,60,600);
}
 */ 

void autonomous() {
  leftDb.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
  rightDb.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  inAuton = true;
  alliance_color = "blue";

  unjam_intake = false;    
  start_time = pros::millis();
  worldsBlueRingSide();
  master.set_text(1, 1, std::to_string(pros::millis() - start_time));



  /***************** BLUE GOAL RUSH 11 x 8.5 **********************/
  /***************** RED GOAL RUSH 11 x 8.5 ***********************/
  
 
 

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  inAuton = false;
  scoring_position = 20500;


  while (true) {
      pros::delay(20);

      // Movement Controls
      double power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
      double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
      leftDb.move(power + turn);
      rightDb.move(power - turn);


      // Motor Brake Modes
      leftDb.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
      rightDb.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
      intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


      // Intake Controls
      if(!currently_unjamming){
          if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){ // Intake
              intake = -127; 
          }
          else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){ // Outtake
              intake = 127;
          }
          else{
              intake = 0;
              pros::delay(10);
          }
      }

      // Mogo Controls
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
          mogo_flag = !mogo_flag;
          if(mogo_flag){
            mogo.set_value(HIGH);
          }
          else{
            mogo.set_value(LOW);
          }
          pros::delay(10);
      }


      // Lift Controls
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
          reset_lift = false;
          unjam_intake = false;
          in_loading_position = true;
          double_ring = false;
          pros::delay(10);
      }

      else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
          in_loading_position = false;
          reset_lift = true;
          unjam_intake = true;
          
          pros::delay(10);
      } 

      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && in_loading_position){
          reset_lift = false;
          in_scoring_position = true;
          double_ring = false;

          pros::delay(10);
      }

      else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) && in_loading_position){
          double_ring = true;

          pros::delay(10);
      }
      else{
          in_scoring_position = false;

          pros::delay(10);
      }
      


      // Sweeper Controls
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !in_loading_position){
          if(!sweeper_flag){
            sweeper_flag = true;
            sweeper.set_value(HIGH);
          }
      }
      else{
          sweeper_flag = false;
          sweeper.set_value(LOW);
          pros::delay(10);
      }


      // Intake Lift Controls
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
          intakelift.set_value(HIGH);
      }
      else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
          intakelift.set_value(LOW);
          pros::delay(10);
      }
  }
}

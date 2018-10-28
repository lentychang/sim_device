/* Modified from...
   wsg_50_keyboard_teleop
 * Copyright (c) 2012, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Lenty Chang (lenty@gmail.com)
 * \brief IIWA keyboard teleop
 */

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>


#define PI 3.14159265359
#define deg 0.01745329251

#define KEYCODE_Q 0x71
#define KEYCODE_A 0x61

#define KEYCODE_E 0x65
#define KEYCODE_D 0x64

#define KEYCODE_W 0x77 
#define KEYCODE_S 0x73

#define KEYCODE_R 0x72
#define KEYCODE_F 0x66

#define KEYCODE_T 0x74
#define KEYCODE_G 0x67

#define KEYCODE_Y 0x79
#define KEYCODE_H 0x68

#define KEYCODE_U 0x75
#define KEYCODE_J 0x6A

#define KEYCODE_SPACEBAR 0x49

#define MAX_ANG PI
#define MIN_ANG -PI


class iiwaTeleop
{
  private:
  double increment, force;
  std_msgs::Float64 cmd1;
  std_msgs::Float64 cmd2;
  std_msgs::Float64 cmd3;
  std_msgs::Float64 cmd4;
  std_msgs::Float64 cmd5;
  std_msgs::Float64 cmd6;
  std_msgs::Float64 cmd7;


  ros::NodeHandle n_;
  ros::Publisher pos_pub_1, pos_pub_2, pos_pub_3, pos_pub_4,
                 pos_pub_5, pos_pub_6, pos_pub_7;

  public:
  void init()
  { 
    cmd1.data = 0;
    cmd2.data = 0;
    cmd3.data = 0;
    cmd4.data = 0;
    cmd5.data = 0;
    cmd6.data = 0;
    cmd7.data = 0;
    float incr = 5.0;

    pos_pub_1 = n_.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J1_controller/command", 1);
    pos_pub_2 = n_.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J2_controller/command", 1);
    pos_pub_3 = n_.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J3_controller/command", 1);
    pos_pub_4 = n_.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J4_controller/command", 1);
    pos_pub_5 = n_.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J5_controller/command", 1);
    pos_pub_6 = n_.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J6_controller/command", 1);
    pos_pub_7 = n_.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J7_controller/command", 1);

    ros::NodeHandle n_private("~");
    n_private.param("increment", increment, incr * deg);
  }
  
  ~iiwaTeleop()   { }
  void keyboardLoop();

};

int kfd = 0;
struct termios cooked, raw;
float currentAng1;
float currentAng2;
float currentAng3;
float currentAng4;
float currentAng5;
float currentAng6;
float currentAng7;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "iiwa_teleop");

  iiwaTeleop tpk;
  tpk.init();

  signal(SIGINT,quit);

  tpk.keyboardLoop();

  return(0);
}

void iiwaTeleop::keyboardLoop()
{
  char c;
  bool dirty[7]= {false, false, false, false, false, false, false};
  currentAng1 = 0.0;
  currentAng2 = 0.0;
  currentAng3 = 0.0;
  currentAng4 = 0.0;
  currentAng5 = 0.0;
  currentAng6 = 0.0;
  currentAng7 = 0.0;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'q, a' to turn joint_1");
  puts("Use 'w, s' to turn joint_2");
  puts("Use 'e, d' to turn joint_3");
  puts("Use 'r, f' to turn joint_4");
  puts("Use 't, g' to turn joint_5");
  puts("Use 'y, h' to turn joint_6");
  puts("Use 'u, j' to turn joint_7");

  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    cmd1.data = currentAng1;
    cmd2.data = currentAng2;
    cmd3.data = currentAng3;
    cmd4.data = currentAng4;
    cmd5.data = currentAng5;
    cmd6.data = currentAng6;
    cmd7.data = currentAng7;

    switch(c) {
      case KEYCODE_Q: // Rotate Joint1 CCW
        if (currentAng1 < MAX_ANG){
          currentAng1 = currentAng1 + increment;
          cmd1.data = currentAng1;
          dirty[0] = true;
          break;
        }
      case KEYCODE_A: // Rotate Joint1 CW
        if (currentAng1 > MIN_ANG){
          currentAng1 = currentAng1 - increment;
          cmd1.data = currentAng1;
          dirty[0] = true;
          break;
        }

      case KEYCODE_W: // Rotate Joint2 CCW
        if (currentAng2 < MAX_ANG){
          currentAng2 = currentAng2 + increment;
          cmd2.data = currentAng2;
          dirty[1] = true;
          break;
        }
      case KEYCODE_S: // Rotate Joint2 CW
        if (currentAng2 > MIN_ANG){
          currentAng2 = currentAng2 - increment;
          cmd2.data = currentAng2;
          dirty[1] = true;
          break;
        }

      case KEYCODE_E: // Rotate Joint3 CCW
        if (currentAng3 < MAX_ANG){
          currentAng3 = currentAng3 + increment;
          cmd3.data = currentAng3;
          dirty[2] = true;
          break;
        }
      case KEYCODE_D: // Rotate Joint3 CW
        if (currentAng3 > MIN_ANG){
          currentAng3 = currentAng3 - increment;
          cmd3.data = currentAng3;
          dirty[2] = true;
          break;
        }
      case KEYCODE_R: // Rotate Joint4 CCW
        if (currentAng4 < MAX_ANG){
          currentAng4 = currentAng4 + increment;
          cmd4.data = currentAng4;
          dirty[3] = true;
          break;
        }
      case KEYCODE_F: // Rotate Joint4 CW
        if (currentAng4 > MIN_ANG){
          currentAng4 = currentAng4 - increment;
          cmd4.data = currentAng4;
          dirty[3] = true;
          break;          
        }

      case KEYCODE_T: // Rotate Joint5 CCW
        if (currentAng5 < MAX_ANG){
          currentAng5 = currentAng5 + increment;
          cmd5.data = currentAng5;
          dirty[4] = true;
          break;
        }
      case KEYCODE_G: // Rotate Joint5 CW
        if (currentAng5 > MIN_ANG){
          currentAng5 = currentAng5 - increment;
          cmd5.data = currentAng5;
          dirty[4] = true;
          break;
        }

      case KEYCODE_Y: // Rotate Joint6 CCW
        if (currentAng6 < MAX_ANG){
          currentAng6 = currentAng6 + increment;
          cmd6.data = currentAng6;
          dirty[5] = true;
          break;
        }
      case KEYCODE_H: // Rotate Joint6 CW
        if (currentAng6 > MIN_ANG){
          currentAng6 = currentAng6 - increment;
          cmd6.data = currentAng6;
          dirty[5] = true;
          break;
        }

      case KEYCODE_U: // Rotate Joint7 CCW
        if (currentAng7 < MAX_ANG){
          currentAng7 = currentAng7 + increment;
          cmd7.data = currentAng7;
          dirty[6] = true;
          break;
        }
      case KEYCODE_J: // Rotate Joint7 CW
        if (currentAng7 > MIN_ANG){
          currentAng7 = currentAng7 - increment;
          cmd7.data = currentAng7;
          dirty[6] = true;
          break;
        }

    }
    
    if (dirty[0] == true) pos_pub_1.publish(cmd1);
    if (dirty[1] == true) pos_pub_2.publish(cmd2);
    if (dirty[2] == true) pos_pub_3.publish(cmd3);
    if (dirty[3] == true) pos_pub_4.publish(cmd4);
    if (dirty[4] == true) pos_pub_5.publish(cmd5);
    if (dirty[5] == true) pos_pub_6.publish(cmd6);
    if (dirty[6] == true) pos_pub_7.publish(cmd7);

  }
}

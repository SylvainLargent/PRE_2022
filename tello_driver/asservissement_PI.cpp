#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include "rrt_lib/Point3.hpp"
#include "rrt_lib/Segment.hpp"
#include "rrt_lib/Env.hpp"
#include "rrt_lib/command.hpp"
#include "rrt_lib/Node.hpp"
#include "rrt_lib/RRT_STAR.hpp"
#include "rrt_lib/RRT.hpp"
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <chrono>

geometry_msgs::PoseStamped coord;
geometry_msgs::PoseStamped coord_cible;

void callBackPosition(const geometry_msgs::PoseStamped msg) {
    coord.pose.position.x = msg.pose.position.x;
    coord.pose.position.y = msg.pose.position.y;
    coord.pose.position.z = msg.pose.position.z;
    //ROS_INFO("Current position: (%g, %g, %g)", coord.x, coord.y, coord.z);
}

void callBackCible(const geometry_msgs::PoseStamped msg) {
    coord_cible.pose.position.x = msg.pose.position.x;
    coord_cible.pose.position.y = msg.pose.position.y;
    coord_cible.pose.position.z = msg.pose.position.z;
    //ROS_INFO("Current position: (%g, %g, %g)", coord.x, coord.y, coord.z);
}


int main(int argc, char ** argv){
    ros::init(argc, argv, "assservissement_PI");
    //Voliere de l'U2IS
    Env environment = Env(-1.5, 1.1, -1.4, 1.8, 0.0, 2, 0.1, 0.1, 0.1);

//Defining the obstacles of the scene
    vector<vector<double>> list_rectangles =
        {   
            {-1.0, -1.25, -0.1, 0.25, 0.7, 0.6}, // Boite proche du mur
            {-1.1, 0.71, -0.1, 0.4, 0.6, 0.5},  //Boite opérateur
            // {-0.6, 0.7, -0.1, 1.25, 1, 3},    //Opérateur
            // {-1.65, -0.2, -0.1, 1.2, 0.3, 3}      //Obstacle virtuel
            {-1.65, -0.75, -0.1, 2.2, 1.1, 3}      //Obstacle virtuel
        };

    
//Adding rectangles to the scene
    for(int i = 0; i < list_rectangles.size(); ++i){
        environment.add_rectangle(list_rectangles[i]);
    }

//Initializing ROS variables correctly
    ros::NodeHandle nh1;
    ros::NodeHandle nh2;
    ros::NodeHandle nh_move;
    ros::NodeHandle nh_pos;
    ros::NodeHandle nh_cible;

    std_msgs::Empty msg1;
    std_msgs::Empty msg2;

    geometry_msgs::Twist cmd;

    ros::Publisher take_off_pub;
    take_off_pub = nh1.advertise<std_msgs::Empty>("/tello/takeoff", 100);
    
    ros::Publisher move_pub ;
    move_pub = nh_move.advertise<geometry_msgs::Twist>("/tello/cmd_vel", 1000);

    ros::Publisher land_pub;
    land_pub = nh2.advertise<std_msgs::Empty>("/tello/land", 100);

    ros::Subscriber pos_sub;
    pos_sub = nh_pos.subscribe<geometry_msgs::PoseStamped>("/mocap_node/Robot_1/pose",1000, callBackPosition);

    ros::Subscriber cible_sub;
    cible_sub = nh_cible.subscribe<geometry_msgs::PoseStamped>("/mocap_node/Cible/pose",1000, callBackCible);

    ros::Rate loop_rate(95);



//Initialization of the various positions by optitrack (call the Callback functions )
    while(coord.pose.position.x == 0){
        ros::spinOnce();
        loop_rate.sleep();
    }

//Planning    
    Point3 * s_start = new Point3(coord.pose.position.x, coord.pose.position.y, coord.pose.position.z+0.75);
    Point3 * s_goal = new Point3(coord_cible.pose.position.x -0.005, coord_cible.pose.position.y - 0.2, (coord_cible.pose.position.z + 0.3));


    //Dans le cas où step_len =  0.25 
    double step_len = 0.25;
    double Kp = 1.8;                       //Coefficient proportionnel du PID
    double Ki = 3.2;                         //Coefficient intégral du PID
    double precision = 0.2;                //Precision au niveau de chaque noeud
    double precision_just_before_landing = 0.18 ;
    double precision_landing = 0.08;       //Precision au niveau de l'arrivée
    double saturation = 0.8;               //Valeur maximale de la commande dans une direction


    //Number of iterations and step length at each step
    int iter_max = 5000;
    double goal_sample_rate = 0.1;
    double search_radius = 3;

    RRT_STAR rrt_algo = RRT_STAR(environment, *s_start, *s_goal, step_len, goal_sample_rate,iter_max, search_radius); 
    
    vector<Node*> path = rrt_algo.planning();  


//In case no path are found 
    ros::spinOnce();
    loop_rate.sleep();
    if(path.size() > 0){
    //TAKE OFF
        sleep(2);
        ROS_INFO("Before takeoff");
        take_off_pub.publish(msg1);
        ROS_INFO("tookoff");
        //set velocity at 0 
        cmd.linear.x = 0;
        cmd.linear.y = 0;
        cmd.linear.z = 0; 
        move_pub.publish(cmd);
    }
    else{
        return 0;
    }

//////////////////////////////////////////////////////// FOR THE PLOT IN PYTHON
//Giving out, arena necessary info !
    ofstream arena_boundaries;
    arena_boundaries.open("/home/boundaries.txt");
    arena_boundaries << environment.get_x_min() << std::endl;
    arena_boundaries << environment.get_x_max() << std::endl;
    arena_boundaries << environment.get_y_min() << std::endl;
    arena_boundaries << environment.get_y_max() << std::endl;
    arena_boundaries << environment.get_z_min() << std::endl;
    arena_boundaries << environment.get_z_max() << std::endl;
    arena_boundaries.close();

    ofstream start_destination;
    start_destination.open("/home/start_destination.txt");
    start_destination << (*rrt_algo.ps_start) << std::endl;
    start_destination << (*rrt_algo.ps_goal) << std::endl;   
    start_destination.close();  
    


//Plotting rectangle obstacles thanks to their vertices
    vector<vector<Node*>> all_boundaries = rrt_algo.env.get_obstacles_vertex();  
    ofstream all_boundaries_stream;
    all_boundaries_stream.open("/home/obstacles.txt");
    if(!all_boundaries.empty()){
        for(int i = 0; i < all_boundaries.size(); ++i){
            for(int j = 0; j < all_boundaries[i].size();++j){
                all_boundaries_stream << (*(all_boundaries[i][j])) << std::endl;
                delete(all_boundaries[i][j]);
            }
        }
    }
    all_boundaries_stream.close();

//Plotting path trajectory
    ofstream path_stream;
    path_stream.open("/home/trajectory.txt");
    if(!path.empty()){
        for(int i = 0; i < path.size(); ++i){
            path_stream << (*path[i]) << std::endl;
        }
        path_stream << rrt_algo.iter_goal << std::endl;  
    }
    path_stream.close();
    if(!path.empty()){
        ROS_INFO("Path length : %f", get_path_length(path));
    }
//Plotting tree 
    vector<Node*> tree = rrt_algo.tree;  
    ofstream tree_stream;
    tree_stream.open("/home/tree.txt");
    if(!tree.empty()){
        for(int i = 0; i < tree.size(); ++i){
            tree_stream << (*tree[i]) << std::endl;
        }
    }
    tree_stream.close();



///////////////////////////////////////////////////////////////////////// END "FOR THE PLOT PYTHON"

//COMMAND
        Point3 position = Point3(coord.pose.position.x, coord.pose.position.y, coord.pose.position.z);
        sleep(1);
        
        int index = path.size()-1;
        Point3 delta_position_current = Point3(0,0,0);
        Point3 Sum_delta_position = Point3(0,0,0);
        Point3 delta_position_previous;

        //Recuperate the command and the position
        ofstream command_pos_stream;
        command_pos_stream.open("/home/last_data.txt");
        auto start = std::chrono::steady_clock::now();

        while (ros::ok()) {
            position = Point3(coord.pose.position.x, coord.pose.position.y, coord.pose.position.z);
            delta_position_previous = delta_position_current;   
            delta_position_current = path[index]->p - position;
            Sum_delta_position += delta_position_current;

            //LANDING
            if(distance(position, *s_goal) < precision){
                Sum_delta_position = Point3(0,0,0);
                while(distance(position, *s_goal) > precision_landing){
                    position = Point3(coord.pose.position.x, coord.pose.position.y, coord.pose.position.z);
                    delta_position_current = path[index]->p - position;
                    Sum_delta_position += delta_position_current;
                    double cmdX = Kp * delta_position_current.x() + Ki * Sum_delta_position.x()  ;
                    double cmdY = Kp * delta_position_current.y() + Ki * Sum_delta_position.y()  ;
                    double cmdZ = Kp * delta_position_current.z() + Ki * Sum_delta_position.z()  ;
                    cmd.linear.x = cmdX;
                    cmd.linear.y = cmdY;
                    cmd.linear.z = cmdZ;
                    move_pub.publish(cmd);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                cmd.linear.x = 0;
                cmd.linear.y = 0;
                cmd.linear.z = 0; 
                move_pub.publish(cmd);  
                land_pub.publish(msg2);
                ROS_INFO("Landed");
                break;
            }
            //New step 
            else if(distance(position, path[index]->p) < precision && index > 0){
                Sum_delta_position = Point3(0,0,0);
                if(distance(path[index-1]->p, path[index]->p) > step_len){
                    //In case the distance between two nodes is too far apart
                    //let's decompose the way between the nodes, so the local command isn't too affected
                    Point3 dir = unit_vector(path[index - 1]->p - path[index]->p);
                    dir *= step_len;    
                    Point3 newObjective = (path[index]->p + dir);
                    path[index] = new Node(newObjective);
                }
                else{
                    index -= 1;   
                }       
                move_pub.publish(cmd);     
            }
            //USUAL COMMAND
            else{
                if(index == 1){
                    precision = precision_just_before_landing;
                }
                double cmdX  = Kp * delta_position_current.x() + Ki * Sum_delta_position.x();
                double cmdY  = Kp * delta_position_current.y() + Ki * Sum_delta_position.y();
                double cmdZ  = Kp * delta_position_current.z() + Ki * Sum_delta_position.z();
                cmd.linear.x =  (abs(cmdX) < saturation) ? cmdX : (saturation * cmdX/abs(cmdX)) ;
                cmd.linear.y =  (abs(cmdY) < saturation) ? cmdY : (saturation * cmdY/abs(cmdY)) ;
                cmd.linear.z =  (abs(cmdZ) < saturation) ? cmdZ : (saturation * cmdZ/abs(cmdZ)) ;
                move_pub.publish(cmd);  
            }
            ROS_INFO("%f, %f, %f", cmd.linear.x, cmd.linear.y, cmd.linear.z);
            ROS_INFO("index : %d and path length : %lu ", index, path.size());
            //Recup data about command and position and time
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = end-start;
            command_pos_stream << cmd.linear.x << "," << cmd.linear.y << "," << cmd.linear.z << "," << coord.pose.position.x << "," << coord.pose.position.y << "," << coord.pose.position.z << "," << path[index]->p.x() << "," << path[index]->p.y() << "," << path[index]->p.z() << "," << index << "," << elapsed_seconds.count() << std::endl;
            loop_rate.sleep();
            ros::spinOnce();
        }

    command_pos_stream.close();
    delete(s_goal);
    delete(s_start);

    return 0;
}

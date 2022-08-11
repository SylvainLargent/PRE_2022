#include <iostream>
#include <cmath>
#include "Point3.hpp"
#include "Segment.hpp"
#include "Env.hpp"
#include "RRT_STAR.hpp"
#include "Node.hpp"
#include <fstream>
#include "ibex.h"

using namespace ibex;

int main(int argc, char ** argv){

    //Voliere de l'U2IS
    Env environment = Env(-1.5, 1.1, -1.4, 1.8, 0.0, 2, 0.25, 0.25, 0.25);

//Defining the obstacles of the scene
    vector<vector<double>> list_rectangles =
        {   
            {-1.0, -1.25, -0.1, 0.25, 0.7, 0.6}, // Boite proche du mur
            {-1.1, 0.71, -0.1, 0.4, 0.6, 0.5},  //Boite opérateur
            // {-0.6, 0.7, -0.1, 1.25, 1, 3},    //Opérateur
            // {-1.65, -0.2, -0.1, 1.2, 0.3, 3}      //Obstacle virtuel
            {-1.15, -0.4, -0.1, 1.15, 0.8, 3}      //Obstacle virtuel
        };

    //Adding rectangles to the scene
    for(int i = 0; i < list_rectangles.size(); ++i){
        environment.add_rectangle(list_rectangles[i]);
    }

    //Obstacles for dynibex
    vector<vector<double>>rectangles_intervals = environment.get_rectangles_boundaries(); //Inflated obstacles boundaries
    
    vector<IntervalVector> obstacles_itv;
    for(int k = 0; k < rectangles_intervals.size(); ++k){
        double boundaries_itv[3][2] = {{rectangles_intervals[k][0],rectangles_intervals[k][1]},
                                        {rectangles_intervals[k][2],rectangles_intervals[k][3]},
                                            {rectangles_intervals[k][4],rectangles_intervals[k][5]}};
        IntervalVector temp = IntervalVector(3, boundaries_itv);
        // std::cout << temp << std::endl;
        obstacles_itv.push_back(temp);
    }

    //Starting point and goal destination
    Point3 s_goal  =  Point3(-0.917, -0.92, 1);
    Point3 s_start  =  Point3(-0.8, 0.80, 1);

    //Number of iterations and step length at each step
    int iter_max = atoi(argv[1]);
    double step_len = atof(argv[2]);
    double search_radius = atof(argv[3]);
    double goal_sample_rate = 0.1;

    

    RRT_STAR rrt_algo = RRT_STAR(environment, s_start, s_goal, step_len, goal_sample_rate, iter_max, search_radius);

    vector<Node*> path = rrt_algo.planning();

/////////////////////////////////////////////////////////////////////PLOT ARENA TRAJECTORY
//Plotting path trajectory
    ofstream path_stream;
    path_stream.open("csv_files/trajectory.txt");
    if(!path.empty()){
        for(int i = 0; i < path.size(); ++i){
            path_stream << (*path[i]) << std::endl;
        }
        path_stream << rrt_algo.iter_goal << std::endl;  
    }
    path_stream.close();
    if(!path.empty()){
        std::cout << "Path length : " << get_path_length(path) << std::endl;
    }

    //Giving out, arena or scene necessary info !
 
    ofstream arena_boundaries;
    arena_boundaries.open("csv_files/boundaries.txt");
    arena_boundaries << environment.get_x_min() << std::endl;
    arena_boundaries << environment.get_x_max() << std::endl;
    arena_boundaries << environment.get_y_min() << std::endl;
    arena_boundaries << environment.get_y_max() << std::endl;
    arena_boundaries << environment.get_z_min() << std::endl;
    arena_boundaries << environment.get_z_max() << std::endl;
    arena_boundaries.close();

    ofstream start_destination;
    start_destination.open("csv_files/start_destination.txt");
    start_destination << (*rrt_algo.ps_start) << std::endl;
    start_destination << (*rrt_algo.ps_goal) << std::endl;   
    start_destination.close();  
    


//Plotting rectangle obstacles thanks to their vertices
    vector<vector<Node*>> all_boundaries = environment.get_obstacles_vertex();  
    ofstream all_boundaries_stream;
    all_boundaries_stream.open("csv_files/obstacles.txt");
    if(!all_boundaries.empty()){
        for(int i = 0; i < all_boundaries.size(); ++i){
            for(int j = 0; j < all_boundaries[i].size();++j){
                all_boundaries_stream << (*(all_boundaries[i][j])) << std::endl;
                delete(all_boundaries[i][j]);
            }
        }
    }
    all_boundaries_stream.close();

//Plotting rectangle obstacles thanks to their vertices considered with the margin of security
    vector<vector<Node*>> all__inflated_boundaries = environment.get_inflated_obs_vertex();  
    ofstream all__inflated_boundaries_stream;
    all__inflated_boundaries_stream.open("csv_files/obstacles_inflated.txt");
    if(!all__inflated_boundaries.empty()){
        for(int i = 0; i < all__inflated_boundaries.size(); ++i){
            for(int j = 0; j < all__inflated_boundaries[i].size();++j){
                all__inflated_boundaries_stream << (*(all__inflated_boundaries[i][j])) << std::endl;
                delete(all__inflated_boundaries[i][j]);
            }
        }
    }
    all__inflated_boundaries_stream.close();

//Plotting tree 
    vector<Node*> tree = rrt_algo.tree;  
    ofstream tree_stream;
    tree_stream.open("csv_files/tree.txt");
    if(!tree.empty()){
        for(int i = 0; i < tree.size(); ++i){
            tree_stream << (*tree[i]) << std::endl;
        }
    }
    tree_stream.close();

//////////////////////////////END PLOT ARENA AND TRAJECTORY
    
    ofstream export_3d;
    export_3d.open("export_3d.txt");
    ifstream command;
    command.open("csv_files/command.txt");
    string line;
    if (command.is_open())
  {
    while ( getline (command,line) )
    {
      cout << line << '\n';
    }
    command.close();
  }
    for(int index = 0; index < path.size()-1; ++index){
        const int n= 6; 
        Variable y(n);
        Affine2Vector state(n);

        IntervalVector yinit(n);
        //defaut sur la mesure Optitrack
        double deltax = 0.0001;
        double deltay = 0.0001;
        double deltaz = 0.0001;
        yinit[0] = Interval(path[index]->p.x() - deltax, path[index]->p.x() + deltax);
        yinit[1] = Interval(path[index]->p.y() - deltay, path[index]->p.y() + deltay);
        yinit[2] = Interval(path[index]->p.z() - deltaz, path[index]->p.z() + deltaz);
        yinit[3] = Interval(0);
        yinit[4] = Interval(0);
        yinit[5] = Interval(0);


        //Paramètres 
        Interval Kp(1.8);
        Interval Ki(3.2);
        Interval Kd(1.5);

        double x_goal = path[index+1]->p.x();
        double y_goal = path[index+1]->p.y();
        double z_goal = path[index+1]->p.z();

        // std::cout << *path[i] << std::endl;
        // std::cout << x_goal << "," << y_goal << "," << z_goal << std::endl; 

        Interval xGoal(x_goal);
        Interval yGoal(y_goal);
        Interval zGoal(z_goal);
        IntervalVector destination_itvs = IntervalVector(6);
        destination_itvs[0] = xGoal + Interval(-0.2, 0.2);
        destination_itvs[1] = yGoal + Interval(-0.2, 0.2);
        destination_itvs[2] = zGoal + Interval(-0.2, 0.2);
        destination_itvs[3] = Interval();
        destination_itvs[4] = Interval();
        destination_itvs[5] = Interval();

        Function ydot = Function(y,Return( 
                            Kp*(xGoal-y[0]) + Ki*y[3],
                            Kp*(yGoal-y[1]) + Ki*y[4],
                            Kp*(zGoal-y[2]) + Ki*y[5],
                            (xGoal - y[0]), //Pour la commande intégrale
                            (yGoal - y[1]),
                            (zGoal - y[2]))    
                        );

        ivp_ode problem = ivp_ode(ydot,0.0,yinit); //Modèle dynamique, temps de départ, yinit

        simulation simu = simulation(&problem,0.5,HEUN,1e-2);

        simu.run_simulation();

//Verification no obstacles touched, and destination reached
        bool no_collision = false;
        for(int k = 0; k < obstacles_itv.size(); ++k){
            if(simu.has_crossed_b(obstacles_itv[k]) == NO){
                no_collision = true;
            }
        }
        
        // std::cout << simu.get_last() << std::endl;
        // std::cout << x_goal << ", " << y_goal << ", " << z_goal <<std::endl;
        std::cout << "Objectives done accordingly ? " << !no_collision  << std::endl;
        std::cout << "Step Destination reached ? " << (simu.finished_in(destination_itvs) == false)  << std::endl;
        

//Write CSV for the 3D boxes for the plot
        std::list<solution_g>::iterator iterator_list;
	    for(iterator_list=simu.list_solution_g.begin();iterator_list!=simu.list_solution_g.end();iterator_list++)
	    {
	      export_3d << iterator_list->box_jnh->operator[](0) <<
		" ; " << iterator_list->box_jnh->operator[](1) <<
		" ; " << iterator_list->box_jnh->operator[](2) << std::endl;
	    }
    }

    export_3d.close(); 
    //For an export in order to plot
    // simu.export3d_yn("export_3d.txt",0,1,2);
    //Instead of exporting the intervals, let's work on it and export the 8 vertices
    //So it can be plotted with python 3D plot


    return 0;

}

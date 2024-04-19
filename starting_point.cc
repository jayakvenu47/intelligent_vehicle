//
// Created by Gastone Pietro Rosati Papini on 10/08/22.
//

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <memory>
#include <iostream>
extern "C" {
#include "screen_print_c.h"
}
#include "screen_print.h"
#include "server_lib.h"
#include "logvars.h"

// --- MATLAB PRIMITIVES INCLUDE ---
// #include "primitives.h"
// --- MATLAB PRIMITIVES INCLUDE ---

#define DEFAULT_SERVER_IP    "127.0.0.1"
#define SERVER_PORT                30000  // Server port
#define DT 0.05

// Handler for CTRL-C
#include <signal.h>
static uint32_t server_run = 1;
void intHandler(int signal) {
    server_run = 0;
}
using namespace std;
struct Pose2d
{
    Pose2d(){};
    Pose2d(float x, float y, float th): x(x), y(y), theta(th) {};
    float x;
    float y;
    float theta;
    float norm2d()
    {
        return sqrt(x*x + y*y);
    };
    float normish()
    {
        return sqrt(x*x + y*y + theta*theta);
    };
    

    Pose2d operator+(Pose2d &b)
    {
        Pose2d c;
        c.x = x + b.x;
        c.y = y + b.y;
        c.theta = theta + b.theta;
        return c;
    };
    Pose2d operator-(Pose2d &b)
    {
        Pose2d c;
        c.x = x - b.x;
        c.y = y - b.y;
        c.theta = theta - b.theta;
        return c;
    };
};

struct Node
{
    std::shared_ptr<Node> parent;
    bool has_parent = false;
    float cost = 0;
    Pose2d pose;
    Node(){};
    Node(Pose2d p) : pose(p){};
};

class Quad
{
    public:
    float xmin;
    float ymin;
    float xmax;
    float ymax;
    int depth;
    const int max_depth = 10;
    vector<shared_ptr<Node>> points;
    vector<Quad> children;

    Quad(){};
    Quad(float xmin, float xmax, float ymin, float ymax, int depth): xmin(xmin), ymin(ymin), xmax(xmax), ymax(ymax), depth(depth)
    {};
    void add_node(shared_ptr<Node> point){
        if (children.size() == 0)
        {
            points.push_back(point);
            if (points.size() > 1 && depth < max_depth) subdivide();
        }
        else add_node_to_children(point);
    };
    void subdivide(){
        float xmid =  (xmin + xmax) / 2;
        float ymid = (ymin + ymax) / 2;
        
        children.push_back(Quad(xmin, ymin, xmid, ymid,depth + 1));
        children.push_back(Quad(xmid, ymin, xmax, ymid,depth + 1));
        children.push_back(Quad(xmin, ymid, xmid, ymax,depth + 1));
        children.push_back(Quad(xmid, ymid, xmax, ymax,depth + 1));
        for (int i = 0; i < points.size(); i++)
        {
            add_node_to_children(points[i]);
        }            
        points.clear();
    };
    void add_node_to_children(shared_ptr<Node> point) {
        float x = point->pose.x;
        float y = point->pose.y;
        for (int i = 0; i < children.size(); i++)
        {    
            if (children[i].xmin <= x < children[i].xmax and children[i].ymin <= y < children[i].ymax)
            {
                children[i].add_node(point);
                break;
            }
        }
    };
    vector<shared_ptr<Node>> get_nearest(Pose2d point, float r)
    {
        vector<shared_ptr<Node>> neighbors;
        find_neighbors_r(point, r, neighbors);
        while( neighbors.size() == 0)
        {
            find_neighbors_r(point, r, neighbors);
            r *= 1.5;
        }
        return neighbors;
    };
    void find_neighbors_r(Pose2d point, float r, vector<shared_ptr<Node>> &neighbors)
    {
        float x = point.x;
        float y = point.y;

        if (children.size() == 0){
    
            for (int i = 0; i < points.size(); i++ )
            {
                float aksjd= points[i]->pose.x;
        
                if ((x-points[i]->pose.x)*(x-points[i]->pose.x) + (y-points[i]->pose.y)*(y-points[i]->pose.y) <= r*r)
                {
                    neighbors.push_back(points[i]);
                }       
            }
        }
        else
        {
    
            for (int i = 0; i < children.size(); i++ )
            {
        
                if (children[i].xmin <= x + r && x - r <= children[i].xmax && children[i].ymin <= y + r && y - r <= children[i].ymax)
                {
            
                    children[i].find_neighbors_r(point, r, neighbors);
                }
                    
            }
        }        

    };

};
struct obstacle{
    Pose2d p;
    float w;
    float l;
    obstacle(float x, float y, float w, float l): p(Pose2d(x,y,0)), w(w), l(l){};
};
class Map
{
    public:

    bool colliding(Pose2d p)
    {
        static double s = 0.5; // safety distance
        for (int i = 0; i < obstacles.size(); i++)
        {
            float x = obstacles[i].p.x;
            float y = obstacles[i].p.y;
            float l2 = obstacles[i].l/2 + s;
            float w2 = obstacles[i].w/2 + s;
            if (p.x > x - w2 && p.x < x + w2 && p.y > y - l2 && p.y < p.y > l2) return true;
        }
        return false;
    };
    vector<obstacle> obstacles;

    void add_obstacle(float x, float y, float l, float w)
    {
        obstacle obs(x,y,w,l);
        obstacles.push_back(obs);
    };

};

class Graph
{
    public:
    Graph() {
        all_nodes = new Quad(0,0,10,10, 10);
    };
    Quad* all_nodes;
    shared_ptr<Node> add_vertex(Pose2d p)
    {
        shared_ptr<Node> n = shared_ptr<Node>(new Node(p));
        all_nodes->add_node(n);
        return n;
    };
    void add_edge(shared_ptr<Node> &p1, shared_ptr<Node> &p2, float cost)
    {
        p2->parent = p1;
        p2->cost = p1->cost + cost;
        p2->has_parent = true;
    };
    vector<shared_ptr<Node>> get_near(Pose2d p, float r)
    {
        return all_nodes->get_nearest(p, r);
    };
    vector<Pose2d> get_path(shared_ptr<Node> &n)
    {
        vector<Pose2d> points;
        points.push_back(n->pose);
        if (n->has_parent)
        {
            printTable("OK", 0);
            vector<Pose2d> points2 = get_path(n->parent);
            printTable("AY", 0);
            points.insert( points.end(), points2.begin(), points2.end() );
            printTable("OKAY", 0);
        } 
        printTable(to_string(points.size()).c_str(), 0);
        return points;
    };
};

class SingleTrack
{
    public:
    Pose2d pose;
    SingleTrack(){};
    float max_steer = M_PI_4;
    float velocity;
    float max_velocity = 20.0;
    float L = 4; // TODO - take from data
    void update(float dt, float steering_angle, float acc)
    {
        velocity += acc *dt;
        velocity = (velocity > max_velocity) ? max_velocity: velocity;
        pose.x += velocity * cos(pose.theta) * dt;
        pose.y += velocity * sin(pose.theta) * dt;
        pose.theta += (velocity / L) * tan(steering_angle) * dt;

    };
};

class RRTstar
{
    public:
    RRTstar(float step_size, bool star): step_size(step_size), star(star) {
        car = SingleTrack();
        car.velocity = 2.0;
        graph = Graph();
        end_node = nullptr;
        };
    Pose2d start;
    Pose2d goal;
    float step_size;
    SingleTrack car;
    Graph graph;
    shared_ptr<Node> end_node;
    bool star;
    Map map;
    Pose2d steer(shared_ptr<Node> near, Pose2d rand)
    {
        Pose2d line = rand - near->pose;
        float heading = atan2(line.y, line.x);
        car.pose = near->pose;
        float steer_angle = heading - car.pose.theta;
        if (abs(steer_angle) > car.max_steer) steer_angle = car.max_steer * steer_angle/abs(steer_angle);
        car.update(step_size/car.velocity, steer_angle, 0);
        return car.pose;
    }
    void rewire(shared_ptr<Node> n)
    {
        vector<shared_ptr<Node>> nearest_set = graph.get_near(n->pose, step_size );
        for (int i = 0; i < nearest_set.size(); i++)
        {
            if (n == nearest_set[i]) continue;
            Pose2d q = steer(n, nearest_set[i]->pose);
            float dist = (n->pose - q).norm2d();
            if ( dist > step_size ) continue;
            if (n->cost + dist < nearest_set[i]->cost) graph.add_edge(n, nearest_set[i], dist);
        } // TODO - make better
    };
    Pose2d get_random_point(float min, float max)
    {
        Pose2d newp;
        newp.x = min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
        newp.y = min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
        newp.theta = -M_PI + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(-M_PI-(2*M_PI))));
        return newp;
    };
    bool colliding(Pose2d p)
    {
        return map.colliding(p);
    };
    vector<Pose2d> planning(int max_steps)
    {
        int s = 0;

        while (s < max_steps)
        {
            s++;
            Pose2d rnd = get_random_point(0,10);
            vector<shared_ptr<Node>> nearest = graph.get_near(rnd, step_size);
            float closest_d = __FLT_MAX__;
            Pose2d new_pose;
            shared_ptr<Node> near_node = nullptr;
            for (int i = 0; i < nearest.size(); i++)
            {
                Pose2d p = steer(nearest[i], rnd);
                if (colliding(p)) continue;
                Pose2d d = p - rnd;
                float dist = d.norm2d();
                if (dist < closest_d){
                    closest_d = dist;
                    new_pose = p;
                    near_node = nearest[i];
                }
            }
            if (closest_d == __FLT_MAX__) continue;
            shared_ptr<Node> new_node = graph.add_vertex(new_pose);
            printTable("so far so good.c..", 0);
            graph.add_edge(near_node, new_node, (near_node->pose - new_node->pose).norm2d());
            printTable("so far so good.d..", 0);

            if (star) rewire(new_node);
            if (is_goal(new_node) && !star) return graph.get_path(new_node);

        }
        printTable("OUT!.", 0);

        return graph.get_path(end_node);                 
        
    };
    bool is_goal(shared_ptr<Node> &n)
    {
        float dist = (goal - n->pose).norm2d();
        printTable(to_string(dist).c_str(), 0);
        if (dist < 2* step_size )
        {
            if (end_node == nullptr) end_node = n;
            if (n->cost < end_node->cost){
                end_node = n;
            }
            return true;
        }
        return false;
    };

};

class LatControl
{
    public:
    float kh = 1.0;
    float kp = 1.0;
    float lookahead_distance = 1.0;
    float control( vector<Pose2d> trajectory, Pose2d car_pose)
    {
        Pose2d lookahead;
        lookahead.x = car_pose.x + lookahead_distance * cos(car_pose.theta);
        lookahead.y = car_pose.y + lookahead_distance * sin(car_pose.theta);
        // # Find the point on the trajectory that is closest to the lookahead distance
        vector<float> dists;
        int idx = 0;
        float best_dist = __FLT_MAX__;
        for (int i = 0; i < trajectory.size(); i++)
        {   
            float dist = (trajectory[i] - lookahead).norm2d();
            if (dist < best_dist)
            {
                dist = best_dist;
                idx = i;
            }
        }

        float path_error = best_dist;
        float val = (float(lookahead.y - car_pose.y) * (trajectory[idx].x - lookahead.x)) - 
           (float(lookahead.x - car_pose.x) * (trajectory[idx].y - lookahead.y));
        float sign = 0;
        if (val > 0) sign = -1;
        else if (val < 0) sign = 1;
        // # Compute the heading error
        float heading_error = trajectory[idx].theta - car_pose.theta;
        // # Compute the steering angle
        float steering_angle = kh * heading_error + kp * path_error * sign;
        // # print(steering_angle)
        return steering_angle;
    }


};

class Localiser
{
    public:
    Pose2d carPose;
    Pose2d obs1PoseOG;
    Pose2d obs2PoseOG;
    float d;
    float xdif;
    float ydif;
    Localiser(){};
    Localiser(Pose2d obs1, Pose2d obs2): obs1PoseOG(obs1), obs2PoseOG(obs2)
    {
        Pose2d dif = obs2 - obs1;
        d = (dif).norm2d();
        xdif = dif.x;
        ydif = dif.y;
        carPose = Pose2d(0,0,0);
    };

    void update_pos(Pose2d obs1, Pose2d obs2, Pose2d expectation)
    {
        float dist1 = obs1.norm2d();
        float dist2 = obs2.norm2d();

        float x = (dist1 * xdif + dist2 * xdif) / d;
        float y = (dist1 * ydif + dist2 * ydif) / d;

        Pose2d candidate1 = obs1PoseOG;
        candidate1.x += x;
        candidate1.y += y;

        Pose2d candidate2 = obs1PoseOG;
        candidate2.x -= x;
        candidate2.y -= y;

        Pose2d new_car_pos = candidate1;
        if ((candidate1-expectation).norm2d() > (candidate2-expectation).norm2d() )
        {
            new_car_pos = candidate2;
        }

        float flat_theta = atan2((obs1PoseOG - new_car_pos).y, (obs1PoseOG - new_car_pos).x);
        float calc_theta = atan2(obs1.y, obs1.x);

        new_car_pos.theta = calc_theta - flat_theta;
        carPose = new_car_pos;
    };
};



int main(int argc, const char * argv[]) 
{
    logger.enable(true);

    // Messages variables
    scenario_msg_t scenario_msg;
    manoeuvre_msg_t manoeuvre_msg;
    size_t scenario_msg_size = sizeof(scenario_msg.data_buffer);
    size_t manoeuvre_msg_size = sizeof(manoeuvre_msg.data_buffer);
    uint32_t message_id = 0;

#ifndef _MSC_VER
    // More portable way of supporting signals on UNIX
    struct sigaction act;
    act.sa_handler = intHandler;
    sigaction(SIGINT, &act, NULL);
#else
    signal(SIGINT, intHandler);
#endif

    server_agent_init(DEFAULT_SERVER_IP, SERVER_PORT);

    // Start server of the Agent
    printLine();
    printLine();
    RRTstar planner = RRTstar(1.0, false);
    planner.map = Map();
    LatControl latCtrl = LatControl();
    Localiser loc(); // Need to init with obstacle positions.
    
    bool got_info = false;

    double integral = 0;
    vector<Pose2d> trajectory;
    Pose2d car_pose;
    car_pose.x = 0;
    car_pose.y = 0;
    car_pose.theta = 0;

    while (server_run == 1) {

        // Clean the buffer
        memset(scenario_msg.data_buffer, '\0', scenario_msg_size);

        // Receive scenario message from the environment
        if (server_receive_from_client(&server_run, &message_id, &scenario_msg.data_struct) == 0) {


            // Data struct
            input_data_str *in = &scenario_msg.data_struct;
            output_data_str *out = &manoeuvre_msg.data_struct;
            out->CycleNumber = in->CycleNumber;
            out->Status = 0;

            double minAcc = -10;
            double maxAcc = 5;

            double time = in->ECUupTime;
            double v0 = in->VLgtFild;
            // double a0 = fmin(in->AlgtFild, minAcc);
            double a0 = in->ALgtFild;
            double lookahead = (v0 * 5 < 50) ? 50 : v0 * 5;
            const double vmin = 3;
            const double vmax = 15;
            double vr = in->RequestedCruisingSpeed;
            

            
            // GET START - current pos
            if (!got_info)
            {
        
                got_info = true;
                planner.start.x = 0;
                planner.start.y = 0;
                planner.start.theta = 0;

                planner.graph.add_vertex(planner.start);
    
                planner.goal.x = 5;
                planner.goal.y = 5;
                planner.goal.theta = 0;

        
                // need to add obstacles to map!!

                printTable("so far so good.1..", 0);
                trajectory = planner.planning(10000);
                printTable("so far so good.2.", 0);

                // TODO 
                // Localiser loc = Localiser(// obstacle poses needed)

                // feed map obstacle pos and sizes.
        
            }

            // TODO 
            // update car pos using localiser - give it poses of obstacles!
            // also need to tune gains for both long and lateral control

            float p_gain = 1.0;
            float i_gain = 1.0;
            float d_gain = 1.0;

            double error = v0 - vr;
            float derivative = a0;
            integral = integral/ 2.0 +  error * DT;
            double req_ped = p_gain * error + i_gain * integral + d_gain * derivative;

            // req_ped = abs(req_ped) < 0.001 ? 0: req_ped;
            // std::cout << '\n' << req_ped << '\n';
            out->RequestedAcc = req_ped;
            out->RequestedSteerWhlAg = latCtrl.control(trajectory, car_pose);
            


            // Send manoeuvre message to the environment
            if (server_send_to_client(server_run, message_id, &manoeuvre_msg.data_struct) == -1) {
                //perror("error send_message()");
                exit(EXIT_FAILURE);
            } else {
                printLogTitle(message_id, "sent message");
            }
        }
    }

    // Close the server of the agent
    server_agent_close();
    return 0;
};

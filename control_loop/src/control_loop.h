#include <iostream>
#include <stdlib.h>
#include "boost/thread.hpp"
#include <fstream>

using namespace std;

class CONTROLLER {
    public:
        CONTROLLER(const double p, const double i, const double d, const double initial_ref);
        CONTROLLER();
        
        void loop();                //Main loop function        
        void system_start();       //start the system
        void set_xdes(double x);   //member to set the desired value
        void set_kp(const double a) {_kp = a;}
        void set_ki(const double a) {_ki = a;}
        void set_kd(const double a) {_kd = a;}
        double get_kp() {return _kp;}
        double get_ki() {return _ki;}
        double get_kd() {return _kd;}
        double get_ref() {return _ref;}
        double get_u() {return _u;}
        double get_t() {return _t;}
        void ask_ref();
        void run();
        void print_on_file();

    private:
        double _ref;    //reference value
        double _kp;     //proportional gain
        double _ki;     //integral gain
        double _kd;     //derivative gain
        double _u;      //control action
        bool _running;  //true if the controller is running
        double _t;    //current execution time
        const double T = 0.01;  //discrete time system period
};



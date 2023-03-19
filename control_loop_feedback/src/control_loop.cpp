#include "control_loop.h"


CONTROLLER::CONTROLLER(const double p, const double i, const double d, const double initial_ref) {
    if(p > 0) _kp = p;
    else _kp = 0;
    if(i > 0) _ki = i;
    else _ki = 0;
    if(d > 0) _kd = d;
    else _kd = 0;
    _ref = initial_ref;
    _u = 0.0;
    _running = false;
    _x = 0.0;
    _t = 0.0;
}

//We can use the class constructor to set parameters
CONTROLLER::CONTROLLER() {
    _kp = 0.0;
    _ki = 0.0;
    _kd = 0.0;
    _ref = 0.0;
    _u = 0.0;
    _running = false;
    _x = 0;
    _t = 0.0;
}


//Sense: get input to change the state of our System
void CONTROLLER::set_xdes(double x) {
    _ref = x;
}


//Random initial value
void CONTROLLER::system_start() {
    _x = 0.1*(rand() % 100 + 1);
    _running = true;
}

void CONTROLLER::loop() {
    
    double e = 0.0; //Error
    double integral = 0.0;
    double e_old = 0.0;
    double derivative = 0.0;

    while(_running) {
        e_old = e;
        e = _ref - _x;
        integral += T*e;
        derivative = (e - e_old)/T;

        _u = _kp*e + _ki*integral + _kd*derivative; //control action

        _x = 1.1*_x + _u; //dummy dynamical system used only for simulation

        _t += T;

        usleep(T*1e6);
    }
    cout << "Controller stopped! \n";
}

void CONTROLLER::ask_ref() {
    double buffer;
    while(_running) {
        cout << "Please enter the new reference value (type any letter to close): \n";
            cin >> buffer;
            if(!cin) {
                cin.clear(); //reset failbit
                cin.ignore(); //flush stream
                cout << "Stopping the controller... \n";
                _running = false;
            }
            else set_xdes(buffer);
            
            usleep(1e6); //sleep for 1 s
    }
}

void CONTROLLER::run() {
    boost::thread loop_t (&CONTROLLER::loop, this);
    boost::thread ask_t (&CONTROLLER::ask_ref, this);
    boost::thread print_on_file_t (&CONTROLLER::print_on_file, this);

    loop_t.join();
    ask_t.join();
    print_on_file_t.join();
}

void CONTROLLER::print_on_file() {
    ofstream file;
    file.open("/tmp/ctrl_output.dat");

    while(_running) {
        file << _u <<' '<< _x << ' ' << _ref << ' ' << _t << endl;
        usleep(T*1e6);
    }

    file.close();
}


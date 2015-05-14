#include <sstream>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <ncurses.h>
#include "VREPClient.hpp"
#include <math.h>
#include "main.hpp"
#include "spline.h"

#define UNUSED(x) (void)(x)

/**
 * VREPClient instance
 * Globale variable
 */
static VREPClient VREP;

using namespace std;

/* Inverse kinematic.
 */
struct angles anglesFromPosition(double x, double y, double z) {
    struct angles result;

    result.theta0 = atan(y/x);
    double r = sqrt(x*x + y*y);
    double AH = r - l0;
    double AM = sqrt(z*z + AH * AH);
    double dis = (AM*AM - l1*l1 - l2*l2) / (2*l1*l2) ;
    if (dis > 1) {
        dis = 1;
    }
    if (dis < -1) {
        dis = -1;
    }
    result.theta2 = acos(dis) + 3 * M_PI / 2;
    double beta = atan(z/AH);
    dis = (l2*l2 - l1*l1 - AM*AM) / (2*l1*AM);
    if (dis > 1) {
        dis = 1;
    }
    if (dis < -1) {
        dis = -1;
    }
    result.theta1 = acos(dis) + beta + M_PI + M_PI / 12;
    return result;
}

void applyAngles(int leg, struct angles angles) {
    VREP.getMotor(3*leg).writePos(angles.theta0);
    VREP.getMotor(3*leg + 1).writePos(angles.theta1);
    VREP.getMotor(3*leg + 2).writePos(angles.theta2);
}

void goTo(double x, double y, double z, int leg) {
    z += l/9;
    switch (leg) {
        case LEG1:
            applyAngles(leg, anglesFromPosition(x, y, z));
            break;
        case LEG2:
            applyAngles(leg, anglesFromPosition(-y, x, z));
            break;
        case LEG3:
            applyAngles(leg, anglesFromPosition(-x, -y, z));
            break;
        case LEG4:
            applyAngles(leg, anglesFromPosition(y, -x, z));
            break;
    }
}

void autreRepere(double xr, double yr, double zr, int leg) {
    double x = xr * cos(-M_PI/4) + yr * sin(-M_PI/4);
    double y = -xr * sin(-M_PI/4) + yr * cos(-M_PI/4);
    goTo(x, y, zr, leg);
}

void autreRepereLEG1(double x, double y, double z) {
    autreRepere(x, -y, z, LEG1);
}

void autreRepereLEG2(double x, double y, double z) {
    autreRepere(x, y, z, LEG2);
}

void autreRepereLEG3(double x, double y, double z) {
    autreRepere(x, -y, z, LEG3);
}

void autreRepereLEG4(double x, double y, double z) {
    autreRepere(x, y, z, LEG4);
}

/* This function make the robot move.
 * We could remap t on one side or the other to make it
 * turn while moving.
 */
void move(double t, double displacement, int k) {
    double dis1 = l/2, dis2 = l/2, dis3 = l/2, dis4 = l/2;
    double a = l/6*cos(k*t)+l/6;
    double b = l/6*cos(k*t+M_PI)+l/6;
    if (a > l/6) {
        a = l/6;
    }
    dis1 = displacement*cos(k*t+M_PI/2)+l/2;
    dis3 = -displacement*cos(k*t+M_PI/2)+l/2;
    if (b > l/6) {
        b = l/6;
    }
    dis2 = displacement*cos(k*t-M_PI/2)+l/2;
    dis4 = -displacement*cos(k*t-M_PI/2)+l/2;

    autreRepereLEG1(dis1, l/2, a);
    autreRepereLEG2(dis2, l/2, b);
    autreRepereLEG3(dis3, l/2, a);
    autreRepereLEG4(dis4, l/2, b);
}

//double abs(double t) {
//    if (t < 0) {
//        t = -t;
//    }
//    return t;
//}

/* We could make this more dynamic...
 */
float x[] = {0, M_PI/4, M_PI/2, 3*M_PI/4, M_PI};
float y[] = {M_PI/6, M_PI/4, M_PI/3, M_PI/4, M_PI/6};

float * spline_init() {
    float *y2 = (float *)malloc(5*sizeof(float));
    spline(x, y, 5, 0, 0, y2);

    return y2;
}

float *y2 = spline_init();

/* I use cubic spline interpolation to remap 
 * the time and the curve that t describe.
 */
double temporal_remapping(double t) {
    float *f_t = (float *)malloc(sizeof(float));
    splint(x, y, y2, 5, t, f_t);

    return *f_t;
}

/* This function makes the robot turn !
 */
void turn(double t, double displacement, int k) {
    double a = l/6*cos(k*t)+l/6;
    double b = l/6*cos(k*t+M_PI)+l/6;
    if (a > l/6) {
        a = l/6;
    }

    if (b > l/6) {
        b = l/6;
    }

    double t1 = temporal_remapping(abs(fmod(k*t/2+M_PI/4, M_PI)));
    double t2 = temporal_remapping(abs(fmod(k*t/2-3*M_PI/4, M_PI)));

    autreRepereLEG1(l*cos(t1)-l/5, l*sin(t1)-l/5, a);
    autreRepereLEG2(l*cos(t2)-l/5, l*sin(t2)-l/5, b);
    autreRepereLEG3(l*cos(t1)-l/5, l*sin(t1)-l/5, a);
    autreRepereLEG4(l*cos(t2)-l/5, l*sin(t2)-l/5, b);
}

/* This function makes the robot dance !
 */
void dance(double t) {
    int k = 4;
    goTo(l/8*cos(k*t)+l/2,      l/8*sin(k*t),           l/8*cos(2.5*t)+l/6, LEG1);
    goTo(l/8*cos(k*t+M_PI),     l/8*sin(k*t+M_PI)-l/2,  l/8*cos(2.5*t)+l/6, LEG2);
    goTo(l/8*cos(k*t)-l/2,      l/8*sin(k*t),           l/8*cos(2.5*t)+l/6, LEG3);
    goTo(l/8*cos(k*t+M_PI),     l/8*sin(k*t+M_PI)+l/2,  l/8*cos(2.5*t)+l/6, LEG4);
}

void exiting() {
    VREP.stop();
    VREP.disconnect();
    exit(0);
}

static void signal_handler(int sig, siginfo_t *siginfo, void *context) {
    UNUSED(sig);
    UNUSED(siginfo);
    UNUSED(context);
    cout << endl << "Exiting..." << endl;
    exiting();
}

static void attachSignalHandler() {
    struct sigaction action;
    bzero(&action, sizeof(action));
    action.sa_sigaction = &signal_handler;
    action.sa_flags = SA_SIGINFO;
    if (sigaction(SIGINT, &action, NULL) < 0) {
        cerr << "Unable to register signal handler" << endl;
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char* argv[]) {
    // Network parameters
    int port = 0;
    char* ip = NULL;

    // Parse input arguments
    if (argc != 3) {
        cerr << "Bad usage. Usage: ./command [ip address] [port number]" << endl;
        cerr << "Provide network parameters to connect to V-REP server" << endl;
        return EXIT_FAILURE;
    } else {
        ip = argv[1];
        port = atoi(argv[2]);
    }

    //Signal attaching
    attachSignalHandler();

    try {
        // Connection to V-REP
        cout << "Connecting to V-REP server " << ip << ":" << port << endl;
        VREP.connect(ip, port);

        // Main Loop
        cout << "Starting simulation" << endl;
        VREP.start();
        VREP.nextStep();

		for (int i = 0; i < 12; i++) {
			cout << i << " " << VREP.getMotor(i).getName() << endl;
		}

        initscr();

        raw();
        keypad(stdscr, TRUE);
        noecho();
        cbreak();
		nodelay(stdscr, TRUE);

        deleteln();
        mvprintw(0, 0, "Use the arrow keys to move the robot, 'd' to dance and 'q' to quit !\n");
        refresh();

        double t = 0;

        while (1) {
            t += 0.050;

			int c = getch();
			flushinp();

            switch (c) {
                case KEY_UP:
                    move(t, l/4, 10);
                    break;
                case KEY_DOWN:
                    move(-t, l/4, 10);
                    break;
                case KEY_LEFT:
                    turn(t, l/4, 10);
                    break;
                case KEY_RIGHT:
                    turn(-t, l/4, 10);
                    break;
                case 'd':
                    dance(t);
                    break;
                case 'q':
                    exiting();
					break;
                default:
                    t = 0;
            }

            VREP.nextStep();
        }

		nocbreak();
        endwin();

        exiting();
    } catch (string str) {
        cerr << "Exception error: " << str << endl;
    }
}

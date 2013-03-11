/*
 *       Example of Fly By waypoint generation.
 *
 *
 *       2013 Code by Brandon Jones. DIYDrones.com
 */


#include <AP_Mission.h>
#include <math.h>
#include <stdarg.h>
#include <GCS_MAVLink.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_Menu.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_L1_Control.h>
#include <DataFlash.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>

AP_HAL::BetterStream* cliSerial;

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#define SERIAL0_BAUD 115200

struct Location temp_read={0};
struct Location temp_jump;
uint8_t *index;

Vector2f test_points[] = {
    Vector2f(-117.041076, 32.844287),
    Vector2f(-117.037579, 32.844269),
    Vector2f(-117.038373, 32.841664),
    Vector2f(-117.035937, 32.840222),
    Vector2f(-117.033448, 32.842854),
    Vector2f(-117.032762, 32.845774),
    Vector2f(-117.039585, 32.846820)
};

#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))

static struct Location location_from_point(Vector2f pt)
{
    struct Location loc = {0};
    loc.lat = pt.y * 1e7;
    loc.lng = pt.x * 1e7;
    return loc;
}

static void test_passed_waypoint(void)
{
    hal.console->printf("waypoint tests starting\n");
    struct Location b1, b2, b3;
    uint8_t dir=0;

    for (uint8_t i=0; i+2<ARRAY_LENGTH(test_points); i++) {
        struct Location Aloc = location_from_point(test_points[i]);
        struct Location Bloc = location_from_point(test_points[i+1]);
        struct Location Cloc = location_from_point(test_points[i+2]);

        if(generate_WP_flyby(20.0, Aloc, Bloc, Cloc, b1, b2, b3, dir)) {
            //hal.console->printf("%u:------------\n", (unsigned)i);

            hal.console->printf("A(1)=%li; ",Aloc.lat);
            hal.console->printf("A(2)=%li; ",Aloc.lng);
            hal.console->printf("B(1)=%li; ",Bloc.lat);
            hal.console->printf("B(2)=%li; ",Bloc.lng);
            hal.console->printf("C(1)=%li; ",Cloc.lat);
            hal.console->printf("C(2)=%li;\n",Cloc.lng);

            hal.console->printf("wpb1(1)=%li; ",b1.lat);
            hal.console->printf("wpb1(2)=%li; ",b1.lng);
            hal.console->printf("wpb2(1)=%li; ",b2.lat);
            hal.console->printf("wpb2(2)=%li; ",b2.lng);
            hal.console->printf("wpb3(1)=%li; ",b3.lat);
            hal.console->printf("wpb3(2)=%li;\n",b3.lng);
            hal.console->printf("plot(C(2),C(1),'o',B(2),B(1),'o',A(2),A(1),'o')\nplot(wpb1(2),wpb1(1),'bx',wpb2(2),wpb2(1),'gx',wpb3(2),wpb3(1),'kx');");
        } else {
            hal.console->printf("%u:------Error---\n", (unsigned)i);
        }
    }
}

void setup(void)
{
    hal.uartA->begin(SERIAL0_BAUD, 128, 128);
    test_passed_waypoint();
}

void loop(void)
{
}

AP_HAL_MAIN();
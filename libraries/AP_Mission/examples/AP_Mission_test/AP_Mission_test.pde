/*
 *       Example of AP_Mission Library.
 *   This example requires a currently loaded mission in your APM.
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
#include <DataFlash.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>

AP_HAL::BetterStream* cliSerial;

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#define SERIAL0_BAUD 115200


AP_Mission mission;
struct Location temp_read={0};
uint8_t *index;

void setup(void)
{
    hal.uartA->begin(SERIAL0_BAUD, 128, 128);

    hal.console->printf("WP_SIZE=%u, WP_START_BYTE=%x\n",WP_SIZE, WP_START_BYTE);

    //Hack because parameters from APM are not loaded
    uint8_t missionsize=14;
    mission.set_command_total(missionsize);

    show_mission(missionsize);
    mission.change_waypoint_index(2);
    sequence_through_mission(missionsize);
    mission.change_waypoint_index(12);
    sequence_through_mission(missionsize);
}

void sequence_through_mission(int missionsize) {
    struct Location tmp_cmd;

    hal.console->printf("------Sequence through mission--------\n");
    index=mission.waypoint_index();
    print_index();

    while(mission.increment_waypoint_index()) {
        index=mission.waypoint_index();
        print_index();
        while(mission.get_new_cmd(tmp_cmd)) {
            hal.console->printf("cmd:"); print_location(tmp_cmd);
            if(tmp_cmd.id == MAV_CMD_DO_JUMP) {
                print_index();
            }

        }
    }
    print_index();
    hal.console->printf("Mission Complete\n");
}

void show_mission(int size){
    hal.console->printf("-----------Displaying Raw Mission-----------\n");
    for(uint16_t i=0; i <size+1; i++) {
        temp_read=mission.get_cmd_with_index_raw(i);
        hal.console->printf("index=%u ", i);
        print_location(temp_read);
    }
    hal.console->printf("------------------End-----------------------\n");
}

void print_location(Location wp){
    hal.console->printf("id=%i, options=%i, lat=%li, lng=%li, alt=%li, p1=%d\n",wp.id, wp.options, wp.lat, wp.lng, wp.alt, wp.p1);
}

void print_index()
{
    hal.console->printf("inx[0]=%u  inx[1]=%u   inx[2]=%u\n",*(index),*(index+1),*(index+2));
}

void loop(void){

}

AP_HAL_MAIN();
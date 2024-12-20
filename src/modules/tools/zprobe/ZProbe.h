/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ZPROBE_H_
#define ZPROBE_H_

#include "Module.h"
#include "Pin.h"

#include <vector>

// defined here as they are used in multiple files
#define zprobe_checksum            CHECKSUM("zprobe")
#define leveling_strategy_checksum CHECKSUM("leveling-strategy")

class StepperMotor;
class Gcode;
class StreamOutput;
class LevelingStrategy;

class ZProbe: public Module
{

public:
    ZProbe() : invert_override(false),invert_probe(false) {};
    virtual ~ZProbe() {};

    void on_module_loaded();
    void on_gcode_received(void *argument);

    bool run_probe(float& mm, float feedrate, float max_dist= -1, bool reverse= false);
    bool run_probe_return(float& mm, float feedrate, float max_dist= -1, bool reverse= false);
    bool doProbeAt(float &mm, float x, float y);

    void coordinated_move(float x, float y, float z, float feedrate, bool relative=false);
    void home();

    bool getProbeStatus() { return this->pin.get(); }
    float getSlowFeedrate() const { return slow_feedrate; }
    float getFastFeedrate() const { return fast_feedrate; }
    float getProbeHeight() const { return probe_height; }
    float getMaxZ() const { return max_z; }

private:
    void config_load();
    bool probe_XYZ(Gcode *gcode);
    void probe_bore(Gcode *gcode);
    void probe_boss(Gcode *gcode);
    void probe_insideCorner(Gcode *gcode);
    void probe_outsideCorner(Gcode *gcode);
    void probe_axisangle(Gcode *gcode);
    void calibrate_probe_bore(Gcode *gcode);
    
    void calibrate_Z(Gcode *gc);
    uint32_t read_probe(uint32_t dummy);
    uint32_t read_calibrate(uint32_t dummy);
    void on_get_public_data(void* argument);
    uint32_t probe_doubleHit(uint32_t dummy);

    float slow_feedrate;
    float fast_feedrate;
    float return_feedrate;
    float probe_height;
    float max_z;
    float dwell_before_probing;

    Pin pin;
    Pin calibrate_pin;
    std::vector<LevelingStrategy*> strategies;
    uint16_t debounce_ms;
	volatile uint16_t debounce, cali_debounce;

    uint32_t probe_trigger_time;

    volatile bool probing;
    volatile bool calibrating;
    volatile bool probe_detected;
    volatile bool calibrate_detected;
    bool bfirstHitDetected  = false;
    bool bNoHited  = false;
    bool bDoubleHited  = false;
    uint32_t probe_hit_time = 0;

    struct {
        bool is_delta:1;
        bool is_rdelta:1;
        bool reverse_z:1;
        bool invert_override:1;
        bool invert_probe:1;
    };
};

#endif /* ZPROBE_H_ */

/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "GcodeDispatch.h"

#include "libs/Kernel.h"
#include "Robot.h"
#include "utils/Gcode.h"
#include "libs/nuts_bolts.h"
#include "modules/robot/Conveyor.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "libs/Logging.h"
#include "libs/FileStream.h"
#include "libs/AppendFileStream.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "utils.h"
#include "LPC17xx.h"
#include "version.h"

// goes in Flash, list of Mxxx codes that are allowed when in Halted state
static const int allowed_mcodes[]= {2,5,9,30,105,114,119,80,81,911,503,106,107}; // get temp, get pos, get endstops etc
static bool is_allowed_mcode(int m) {
    for (size_t i = 0; i < sizeof(allowed_mcodes)/sizeof(int); ++i) {
        if(allowed_mcodes[i] == m) return true;
    }
    return false;
}

void GcodeDispatch::init()
{
    modal_group_1= 0;
}

// Called when the module has just been loaded
void GcodeDispatch::on_module_loaded()
{
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
}

// When a command is received, if it is a Gcode, dispatch it as an object via an event
void GcodeDispatch::on_console_line_received(void *line)
{
    SerialMessage new_message = *static_cast<SerialMessage *>(line);
    string possible_command = new_message.message;

    // just reply ok to empty lines
    if(possible_command.empty()) {
        new_message.stream->printf("ok\r\n");
        return;
    }

    // get rid of spaces
    ltrim(possible_command);

try_again:

    char first_char = possible_command[0];
    unsigned int n;

    if (first_char == '$') {
        // ignore as simpleshell will handle it
        return;

    } else if(islower(first_char)) {
        // ignore all lowercase as they are simpleshell commands
        return;
    }

    if ( first_char == 'G' || first_char == 'M' || first_char == 'T' || first_char == 'S' || first_char == 'N' || first_char == '#') {

        //Get linenumber
        if ( first_char == 'N' ) {
            //Strip line number value from possible_command
			size_t lnsize = possible_command.find_first_not_of("N0123456789.,- ");
			if(lnsize != string::npos) {
				possible_command = possible_command.substr(lnsize);
			}else{
				// it is a blank line
				possible_command.clear();
			}
        }

        if ( first_char == 'G'){
			//check if has G90/G91
			std::string g90_g91_command;
			std::string modified_command = possible_command;
			size_t g90_pos = modified_command.find("G90");
			size_t g91_pos = modified_command.find("G91");
			// if we have G90 or G91，then we move G90/G91 to the beginning
			if (g90_pos != std::string::npos) {
				g90_g91_command = "G90";
				modified_command.erase(g90_pos, 3); // delete "G90"
				possible_command = g90_g91_command + modified_command;
			} else if (g91_pos != std::string::npos) {
				g90_g91_command = "G91";
				modified_command.erase(g91_pos, 3); // delete "G91"
				possible_command = g90_g91_command + modified_command;
			}
		}

        //Remove comments
        size_t comment = possible_command.find_first_of(";(");
        if( comment != string::npos ) {
            possible_command = possible_command.substr(0, comment);
        }

		string single_command;
		size_t cmd_pos = string::npos;
		while (possible_command.size() > 0) {
			// assumes G or M are always the first on the line
			// -> G or M are in the line but not always the first char
			// -> S or T could be in front of or after M
			first_char = possible_command[0];
			if (first_char == 'G') {
				// find next G/M/S/T
				if (possible_command.find_first_of("S", 2) != string::npos
						&& possible_command.find_first_of("M", 2) != string::npos) {
					cmd_pos = possible_command.find_first_of("GMST", 2);
				} else {
					cmd_pos = possible_command.find_first_of("GMT", 2);
				}
			} else if (first_char == 'M') {
				// find next G/M
				cmd_pos = possible_command.find_first_of("GM", 2);
			} else if (first_char == 'T' || first_char == 'S') {
				// find first M
				cmd_pos = possible_command.find_first_of("M", 2);
				if (cmd_pos == string::npos) {
					// find first G/S/T
					cmd_pos = possible_command.find_first_of("GST", 2);
				} else {
					// M found, find second G/M/S/T
					cmd_pos = possible_command.find_first_of("GMST", cmd_pos + 2);
				}
			}

			if (cmd_pos == string::npos) {
				single_command = possible_command;
				possible_command = "";
			} else {
				single_command = possible_command.substr(0, cmd_pos);
				possible_command = possible_command.substr(cmd_pos);
			}

				// Prepare gcode for dispatch
			// new_message.stream->printf("GCode1: %s!\n", single_command.c_str());
			Gcode *gcode = new Gcode(single_command, new_message.stream, false, new_message.line);

			if ( first_char == '#'){
				gcode->set_variable_value();
			}

			if ( first_char == '#'){
				gcode->set_variable_value();
			}

			if(THEKERNEL.is_halted()) {
				// we ignore all commands until M999, unless it is in the exceptions list (like M105 get temp)
				if(gcode->has_m && gcode->m == 999) {
					if(THEKERNEL.is_halted()) {
						THEKERNEL.call_event(ON_HALT, (void *)1); // clears on_halt
						new_message.stream->printf("WARNING: After HALT you should HOME as position is currently unknown\n");
					}
					new_message.stream->printf("ok\n");
					delete gcode;
					return;

				}else if(!is_allowed_mcode(gcode->m)) {
					// ignore everything, return error string to host
					new_message.stream->printf("error:Alarm lock\n");
					delete gcode;
					return;
				}
			}

			if(gcode->has_g) {
				if(gcode->g == 53) { // G53 makes next movement command use machine coordinates
					// this is ugly to implement as there may or may not be a G0/G1 on the same line
					// valid version seem to include G53 G0 X1 Y2 Z3 G53 X1 Y2
					if(possible_command.empty()) {
						// use last gcode G1 or G0 if none on the line, and pass through as if it was a G0/G1
						// TODO it is really an error if the last is not G0 thru G3
						if(modal_group_1 > 3) {
							delete gcode;
							new_message.stream->printf("ok - Invalid G53\r\n");
							return;
						}
						// use last G0 or G1
						gcode->g= modal_group_1;

					}else{
						delete gcode;
						// extract next G0/G1 from the rest of the line, ignore if it is not one of these
						gcode = new Gcode(possible_command, new_message.stream);
						possible_command= "";
						if(!gcode->has_g || gcode->g > 1) {
							// not G0 or G1 so ignore it as it is invalid
							delete gcode;
							new_message.stream->printf("ok - Invalid G53\r\n");
							return;
						}
					}
					// makes it handle the parameters as a machine position
					THEROBOT.next_command_is_MCS= true;
				}

				// remember last modal group 1 code
				if(gcode->g < 4) {
					modal_group_1= gcode->g;
				}
			}

			// new_message.stream->printf("dispatch gcode command: '%s' G%d M%d...", gcode->get_command(), gcode->g, gcode->m);
			//Dispatch message!
			THEKERNEL.call_event(ON_GCODE_RECEIVED, gcode );

			if (gcode->is_error) {
				// report error
				new_message.stream->printf("error:");

				if(!gcode->txt_after_ok.empty()) {
					new_message.stream->printf("%s\r\n", gcode->txt_after_ok.c_str());
					gcode->txt_after_ok.clear();

				}else{
					new_message.stream->printf("unknown\r\n");
				}

				// we cannot continue safely after an error so we enter HALT state
				new_message.stream->printf("Entering Alarm/Halt state\n");
				THEKERNEL.call_event(ON_HALT, nullptr);

			} else {

				if(gcode->add_nl)
					new_message.stream->printf("\r\n");

				if(!gcode->txt_after_ok.empty()) {
					new_message.stream->printf("ok %s\r\n", gcode->txt_after_ok.c_str());
					gcode->txt_after_ok.clear();

				} else {
					if(THEKERNEL.is_ok_per_line()) {
						// only send ok once per line if this is a multi g code line send ok on the last one
						if(possible_command.empty())
							new_message.stream->printf("ok\r\n");
					} else {
						// maybe should do the above for all hosts?
						new_message.stream->printf("ok\r\n");
					}
				}
			}		

			delete gcode;
		}
    } else if ( first_char == ';' || first_char == '(' || first_char == '\n' || first_char == '\r' ) {
        // Ignore comments and blank lines
        new_message.stream->printf("ok\n");

    } else if( (n=possible_command.find_first_of("XYZAF")) == 0 || (first_char == ' ' && n != string::npos) ) {
        // handle pycam syntax, use last modal group 1 command and resubmit if an X Y Z or F is found on its own line
        char buf[6];
        if(possible_command[n] == 'F') {
            // F on its own always applies to G1
            strcpy(buf,"G1 ");
        }else{
            // use last modal command (G1 or G0 etc)
            snprintf(buf, sizeof(buf), "G%d ", modal_group_1);
        }
        possible_command.insert(0, buf);
        goto try_again;


    } else {
        // an uppercase non command word on its own (except XYZAF) just returns ok, we could add an error but no hosts expect that.
        new_message.stream->printf("ok - ignore: [%s]\n", possible_command.c_str());
    }
}


#ifndef TIE_ROBOT_CONTROL_MODUAN_ERROR_HANDLING_HPP
#define TIE_ROBOT_CONTROL_MODUAN_ERROR_HANDLING_HPP

#include <string>

void handle_system_error(const std::string& error_msg);
void clear_system_error();
void signalHandler(int signum);

#endif

#ifndef TIE_ROBOT_CONTROL_MODUAN_REGISTER_MAP_HPP
#define TIE_ROBOT_CONTROL_MODUAN_REGISTER_MAP_HPP

namespace tie_robot_control::moduan_registers {

constexpr int LIGHT = 5084;

constexpr int WX_SPEED = 5050;
constexpr int WY_SPEED = 5054;
constexpr int WZ_SPEED = 5058;
constexpr int WX_COORDINATE = 5062;
constexpr int WY_COORDINATE = 5066;
constexpr int WZ_COORDINATE = 5070;

constexpr int X_COORDINATE_ONE = 5510;
constexpr int Y_COORDINATE_ONE = 5512;
constexpr int Z_COORDINATE_ONE = 5514;
constexpr int RZ_COORDINATE_ONE = 5516;
constexpr int X_COORDINATE_TWO = 5520;
constexpr int Y_COORDINATE_TWO = 5522;
constexpr int Z_COORDINATE_TWO = 5524;
constexpr int RZ_COORDINATE_TWO = 5526;
constexpr int X_COORDINATE_THREE = 5530;
constexpr int Y_COORDINATE_THREE = 5532;
constexpr int Z_COORDINATE_THREE = 5534;
constexpr int RZ_COORDINATE_THREE = 5536;
constexpr int X_COORDINATE_FOUR = 5540;
constexpr int Y_COORDINATE_FOUR = 5542;
constexpr int Z_COORDINATE_FOUR = 5544;
constexpr int RZ_COORDINATE_FOUR = 5546;
constexpr int X_COORDINATE_FIVE = 5550;
constexpr int Y_COORDINATE_FIVE = 5552;
constexpr int Z_COORDINATE_FIVE = 5554;
constexpr int RZ_COORDINATE_FIVE = 5556;
constexpr int X_COORDINATE_SIX = 5560;
constexpr int Y_COORDINATE_SIX = 5562;
constexpr int Z_COORDINATE_SIX = 5564;
constexpr int RZ_COORDINATE_SIX = 5566;
constexpr int X_COORDINATE_SEVEN = 5570;
constexpr int Y_COORDINATE_SEVEN = 5572;
constexpr int Z_COORDINATE_SEVEN = 5574;
constexpr int RZ_COORDINATE_SEVEN = 5576;
constexpr int X_COORDINATE_EIGHT = 5580;
constexpr int Y_COORDINATE_EIGHT = 5582;
constexpr int Z_COORDINATE_EIGHT = 5584;
constexpr int RZ_COORDINATE_EIGHT = 5586;
constexpr int X_COORDINATE_NINE = 5590;
constexpr int Y_COORDINATE_NINE = 5592;
constexpr int Z_COORDINATE_NINE = 5594;
constexpr int RZ_COORDINATE_NINE = 5596;

constexpr int MODULE_STOP = 6456;
constexpr int EN_DISABLE = 5076;
constexpr int WARNING_RESET = 5077;
constexpr int LASHING = 5078;
constexpr int WRITING_ANGLE = 5080;
constexpr int WRITING_RMOTOR_SPEED = 5082;
constexpr int EN_DISABLE_RMOTOR = 5087;
constexpr int RESET_RMOTOR = 5088;
constexpr int CEJU = 5068;
constexpr int ARRIVEZ = 5069;
constexpr int FINISHALL = 5210;

constexpr int RX_SPEED = 5150;
constexpr int RY_SPEED = 5154;
constexpr int RZ_SPEED = 5158;
constexpr int RX_COORDINATE = 5162;
constexpr int RY_COORDINATE = 5166;
constexpr int RZ_COORDINATE = 5170;
constexpr int IS_ZERO = 5072;
constexpr int IS_STOP = 5074;
constexpr int IS_ERROR = 5174;
constexpr int IS_LASHING = 5173;
constexpr int ERROR_INQUIRE = 5175;
constexpr int MODULE_STATUS = 5176;
constexpr int EMERGENCY_STOP = 5177;
constexpr int READING_ANGLE = 5178;
constexpr int READING_RMOTOR_SPEED = 5182;
constexpr int BATTERY_VOLTAGE = 5187;
constexpr int INNER_TEM = 5191;
constexpr int X_GESTURE = 5196;
constexpr int Y_GESTURE = 5198;

constexpr int AXIS_X = 0;
constexpr int AXIS_Y = 3;
constexpr int AXIS_Z = 4;
constexpr int AXIS_MOTOR = 5;

constexpr int kPointSlotCount = 9;

static constexpr int kXCoordinateSlots[kPointSlotCount] = {
    X_COORDINATE_ONE,
    X_COORDINATE_TWO,
    X_COORDINATE_THREE,
    X_COORDINATE_FOUR,
    X_COORDINATE_FIVE,
    X_COORDINATE_SIX,
    X_COORDINATE_SEVEN,
    X_COORDINATE_EIGHT,
    X_COORDINATE_NINE,
};

static constexpr int kYCoordinateSlots[kPointSlotCount] = {
    Y_COORDINATE_ONE,
    Y_COORDINATE_TWO,
    Y_COORDINATE_THREE,
    Y_COORDINATE_FOUR,
    Y_COORDINATE_FIVE,
    Y_COORDINATE_SIX,
    Y_COORDINATE_SEVEN,
    Y_COORDINATE_EIGHT,
    Y_COORDINATE_NINE,
};

static constexpr int kZCoordinateSlots[kPointSlotCount] = {
    Z_COORDINATE_ONE,
    Z_COORDINATE_TWO,
    Z_COORDINATE_THREE,
    Z_COORDINATE_FOUR,
    Z_COORDINATE_FIVE,
    Z_COORDINATE_SIX,
    Z_COORDINATE_SEVEN,
    Z_COORDINATE_EIGHT,
    Z_COORDINATE_NINE,
};

static constexpr int kRzCoordinateSlots[kPointSlotCount] = {
    RZ_COORDINATE_ONE,
    RZ_COORDINATE_TWO,
    RZ_COORDINATE_THREE,
    RZ_COORDINATE_FOUR,
    RZ_COORDINATE_FIVE,
    RZ_COORDINATE_SIX,
    RZ_COORDINATE_SEVEN,
    RZ_COORDINATE_EIGHT,
    RZ_COORDINATE_NINE,
};

}  // namespace tie_robot_control::moduan_registers

#endif

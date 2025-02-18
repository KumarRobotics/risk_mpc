# Robot Power readings

std_msgs/Header header

# AC Power
int8 NOT_APPLICABLE=-1

int8 shore_power_connected  # Indicates if AC power is connected.
int8 battery_connected      # Indicates if battery is connected.
int8 power_12v_user_nominal # Indicates if the 12V user power is good.
int8 charger_connected      # Indicates if a charger is connected.
int8 charging_complete      # Indicates if charging is complete.

# Voltage rails, in volts
# Averaged over the message period

# Jackal (J100)
uint8 J100_MEASURED_BATTERY=0
uint8 J100_MEASURED_5V=1
uint8 J100_MEASURED_12V=2

# Dingo 1.0 (D100)
uint8 D100_MEASURED_BATTERY=0
uint8 D100_MEASURED_5V=1
uint8 D100_MEASURED_12V=2

# Dingo 1.5 (D150)
uint8 D150_MEASURED_BATTERY=0
uint8 D150_MEASURED_5V=1
uint8 D150_MEASURED_12V=2

# Warthog (W200)
uint8 W200_MEASURED_BATTERY=0
uint8 W200_MEASURED_12V=1
uint8 W200_MEASURED_24V=2
uint8 W200_MEASURED_48V=3

# Ridgeback (R100)
uint8 R100_MEASURED_BATTERY=0
uint8 R100_MEASURED_5V=1
uint8 R100_MEASURED_12V=2
uint8 R100_MEASURED_INVERTER=3
uint8 R100_MEASURED_FRONT_AXLE=4
uint8 R100_MEASURED_REAR_AXLE=5
uint8 R100_MEASURED_LIGHT=6

# Husky (A200)
uint8 A200_BATTERY_VOLTAGE=0
uint8 A200_LEFT_DRIVER_VOLTAGE=1
uint8 A200_RIGHT_DRIVER_VOLTAGE=2
uint8 A200_VOLTAGES_SIZE=3

float32[] measured_voltages

# Current senses available on platform, in amps.
# Averaged over the message period

# Jackal (J100)
uint8 J100_TOTAL_CURRENT=0
uint8 J100_COMPUTER_CURRENT=1
uint8 J100_DRIVE_CURRENT=2
uint8 J100_USER_CURRENT=3

# Dingo 1.0 (D100)
uint8 D100_TOTAL_CURRENT=0
uint8 D100_COMPUTER_CURRENT=1

# Dingo 1.5 (D150)
uint8 D150_TOTAL_CURRENT=0
uint8 D150_COMPUTER_CURRENT=1

# Warthog (W200)
uint8 W200_TOTAL_CURRENT=0
uint8 W200_COMPUTER_CURRENT=1
uint8 W200_12V_CURRENT=2
uint8 W200_24V_CURRENT=3

# Ridgeback (R100)
uint8 R100_TOTAL_CURRENT=0

# Husky (A200)
uint8 A200_MCU_AND_USER_PORT_CURRENT=0
uint8 A200_LEFT_DRIVER_CURRENT=1
uint8 A200_RIGHT_DRIVER_CURRENT=2
uint8 A200_CURRENTS_SIZE=3

float32[] measured_currents

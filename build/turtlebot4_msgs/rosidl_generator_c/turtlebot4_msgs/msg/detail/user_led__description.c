// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from turtlebot4_msgs:msg/UserLed.idl
// generated code does not contain a copyright notice

#include "turtlebot4_msgs/msg/detail/user_led__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_turtlebot4_msgs
const rosidl_type_hash_t *
turtlebot4_msgs__msg__UserLed__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x33, 0xf0, 0x41, 0xca, 0x1b, 0x65, 0x23, 0xe9,
      0x96, 0xc5, 0xb1, 0xf6, 0x24, 0x4d, 0xe8, 0x90,
      0xfa, 0xae, 0xe9, 0x10, 0x60, 0xb6, 0xf9, 0x68,
      0x40, 0x0d, 0xaa, 0x46, 0x87, 0xa9, 0x02, 0x08,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char turtlebot4_msgs__msg__UserLed__TYPE_NAME[] = "turtlebot4_msgs/msg/UserLed";

// Define type names, field names, and default values
static char turtlebot4_msgs__msg__UserLed__FIELD_NAME__led[] = "led";
static char turtlebot4_msgs__msg__UserLed__FIELD_NAME__color[] = "color";
static char turtlebot4_msgs__msg__UserLed__FIELD_NAME__blink_period[] = "blink_period";
static char turtlebot4_msgs__msg__UserLed__FIELD_NAME__duty_cycle[] = "duty_cycle";

static rosidl_runtime_c__type_description__Field turtlebot4_msgs__msg__UserLed__FIELDS[] = {
  {
    {turtlebot4_msgs__msg__UserLed__FIELD_NAME__led, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot4_msgs__msg__UserLed__FIELD_NAME__color, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot4_msgs__msg__UserLed__FIELD_NAME__blink_period, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot4_msgs__msg__UserLed__FIELD_NAME__duty_cycle, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
turtlebot4_msgs__msg__UserLed__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {turtlebot4_msgs__msg__UserLed__TYPE_NAME, 27, 27},
      {turtlebot4_msgs__msg__UserLed__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This message sets the state of the user LEDs\n"
  "# Blink period is the time in milliseconds during which the ON/OFF cycle occurs\n"
  "# The duty cycle represents the percentage of the blink period that the LED is ON\n"
  "# A duty cycle of 1.0 would set the LED to always be ON, whereas a duty cycle of 0.0 is always OFF\n"
  "# A blink period of 1000ms with a duty cycle of 0.6 will have the LED turn ON for 600ms, \n"
  "# then OFF for 400ms\n"
  "\n"
  "# Available LEDs\n"
  "uint8 USER_LED_1 = 0\n"
  "uint8 USER_LED_2 = 1\n"
  "\n"
  "# Available colors\n"
  "uint8 COLOR_OFF = 0\n"
  "uint8 COLOR_GREEN = 1\n"
  "uint8 COLOR_RED = 2\n"
  "uint8 COLOR_YELLOW = 3 \n"
  "\n"
  "\n"
  "# Which available LED to use\n"
  "uint8 led\n"
  "\n"
  "# Which color to set the LED to\n"
  "uint8 color\n"
  "\n"
  "# Blink period in ms\n"
  "uint32 blink_period\n"
  "\n"
  "# Duty cycle (0.0 to 1.0)\n"
  "float64 duty_cycle";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
turtlebot4_msgs__msg__UserLed__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {turtlebot4_msgs__msg__UserLed__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 758, 758},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
turtlebot4_msgs__msg__UserLed__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *turtlebot4_msgs__msg__UserLed__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from turtlebot4_msgs:msg/UserDisplay.idl
// generated code does not contain a copyright notice

#include "turtlebot4_msgs/msg/detail/user_display__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_turtlebot4_msgs
const rosidl_type_hash_t *
turtlebot4_msgs__msg__UserDisplay__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x82, 0x3b, 0xd5, 0x81, 0xb3, 0x74, 0x91, 0x81,
      0x16, 0x78, 0x8c, 0x92, 0x68, 0x75, 0x21, 0xb1,
      0xbb, 0x8e, 0xa3, 0x33, 0xd8, 0x5d, 0xc1, 0xd4,
      0x0d, 0x60, 0x0c, 0xdd, 0x7d, 0xaf, 0x26, 0x65,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char turtlebot4_msgs__msg__UserDisplay__TYPE_NAME[] = "turtlebot4_msgs/msg/UserDisplay";

// Define type names, field names, and default values
static char turtlebot4_msgs__msg__UserDisplay__FIELD_NAME__ip[] = "ip";
static char turtlebot4_msgs__msg__UserDisplay__FIELD_NAME__battery[] = "battery";
static char turtlebot4_msgs__msg__UserDisplay__FIELD_NAME__entries[] = "entries";
static char turtlebot4_msgs__msg__UserDisplay__FIELD_NAME__selected_entry[] = "selected_entry";

static rosidl_runtime_c__type_description__Field turtlebot4_msgs__msg__UserDisplay__FIELDS[] = {
  {
    {turtlebot4_msgs__msg__UserDisplay__FIELD_NAME__ip, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot4_msgs__msg__UserDisplay__FIELD_NAME__battery, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot4_msgs__msg__UserDisplay__FIELD_NAME__entries, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_ARRAY,
      5,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot4_msgs__msg__UserDisplay__FIELD_NAME__selected_entry, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
turtlebot4_msgs__msg__UserDisplay__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {turtlebot4_msgs__msg__UserDisplay__TYPE_NAME, 31, 31},
      {turtlebot4_msgs__msg__UserDisplay__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This message represents the header and 5 entries \n"
  "# that are displayed on the Turtlebot4 display\n"
  "# selected_entry indicates which menu entry is currently selected\n"
  "\n"
  "string ip\n"
  "string battery\n"
  "string[5] entries\n"
  "int32 selected_entry";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
turtlebot4_msgs__msg__UserDisplay__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {turtlebot4_msgs__msg__UserDisplay__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 229, 229},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
turtlebot4_msgs__msg__UserDisplay__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *turtlebot4_msgs__msg__UserDisplay__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

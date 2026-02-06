// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from turtlebot4_msgs:msg/UserButton.idl
// generated code does not contain a copyright notice

#include "turtlebot4_msgs/msg/detail/user_button__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_turtlebot4_msgs
const rosidl_type_hash_t *
turtlebot4_msgs__msg__UserButton__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe0, 0x74, 0x47, 0xea, 0xba, 0xdb, 0x14, 0xc0,
      0x8c, 0x03, 0xe8, 0xe9, 0x2c, 0x43, 0x48, 0x91,
      0xa6, 0x22, 0x0a, 0x72, 0xd7, 0x00, 0x76, 0x60,
      0x12, 0xeb, 0xc0, 0x7e, 0x69, 0x21, 0x22, 0x36,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char turtlebot4_msgs__msg__UserButton__TYPE_NAME[] = "turtlebot4_msgs/msg/UserButton";

// Define type names, field names, and default values
static char turtlebot4_msgs__msg__UserButton__FIELD_NAME__button[] = "button";

static rosidl_runtime_c__type_description__Field turtlebot4_msgs__msg__UserButton__FIELDS[] = {
  {
    {turtlebot4_msgs__msg__UserButton__FIELD_NAME__button, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
turtlebot4_msgs__msg__UserButton__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {turtlebot4_msgs__msg__UserButton__TYPE_NAME, 30, 30},
      {turtlebot4_msgs__msg__UserButton__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This message relays the state of the user buttons\n"
  "# Each button is represented with a boolean, were True indicates the button is pressed\n"
  "\n"
  "bool[4] button";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
turtlebot4_msgs__msg__UserButton__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {turtlebot4_msgs__msg__UserButton__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 154, 154},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
turtlebot4_msgs__msg__UserButton__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *turtlebot4_msgs__msg__UserButton__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
